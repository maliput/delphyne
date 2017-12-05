// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "backend/simulation_runner.h"

namespace delphyne {
namespace backend {

/// \brief Flag to detect SIGINT or SIGTERM while the code is executing
/// WaitForShutdown().
static bool g_shutdown = false;

/// \brief Mutex to protect the boolean shutdown variable.
static std::mutex g_shutdown_mutex;

/// \brief Condition variable to wakeup WaitForShutdown() and exit.
static std::condition_variable g_shutdown_cv;

//////////////////////////////////////////////////
/// \brief Function executed when a SIGINT or SIGTERM signals are captured.
/// \param[in] _signal Signal received.
static void SignalHandler(const int _signal) {
  if (_signal == SIGINT || _signal == SIGTERM) {
    {
      std::unique_lock<std::mutex> lk(g_shutdown_mutex);
      g_shutdown = true;
    }
    g_shutdown_cv.notify_all();
  }
}

//////////////////////////////////////////////////
void WaitForShutdown() {
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::unique_lock<std::mutex> lk(g_shutdown_mutex);
  g_shutdown_cv.wait(lk, [] { return g_shutdown; });
}

/////////////////////////////////////////////////
SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> _sim,
    double _timeStep)
    : timeStep(_timeStep), simulator(std::move(_sim)) {
  // Advertise the service for controlling the simulation.
  this->node.Advertise(this->kControlService, &SimulatorRunner::OnWorldControl,
                       this);

  // Advertise the topic for publishing notifications.
  this->notificationsPub = this->node.Advertise<ignition::msgs::WorldControl>(
      this->kNotificationsTopic);

  // TODO(basicNew): This is just an initial implementation and is *not* the
  // way we are going to be handling these kind of requests in the future. The
  // process of handling a service calls that needs a response is:
  // - Create a new request and add it to the queue for later processing. As
  // part of that request we should have a topic name where we will post the
  // response.
  // - When the request is processed, the model will be fetched from the
  // simulator and posted to the requested topic.

  this->node.Advertise("/GetRobotModel", &SimulatorRunner::OnRobotModelRequest,
                       this);

  this->simulator->Start();
}

//////////////////////////////////////////////////
SimulatorRunner::~SimulatorRunner() {
  {
    // Tell the main loop thread to terminate.
    std::lock_guard<std::mutex> lock(this->mutex);
    this->enabled = false;
  }
  if (this->mainThread.joinable()) {
    this->mainThread.join();
  }
}

//////////////////////////////////////////////////
void SimulatorRunner::Start() {
  // The main loop is already running.
  if (this->enabled) return;

  this->enabled = true;

  // Start the thread that receives discovery information.
  this->mainThread = std::thread(&SimulatorRunner::Run, this);
}

//////////////////////////////////////////////////
void SimulatorRunner::Run() {
  bool stayAlive = true;
  while (stayAlive) {
    // Start a timer to measure the time we spend doing tasks.
    auto stepStart = std::chrono::steady_clock::now();

    // 1. Process incoming messages (requests).
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->ProcessIncomingMessages();
    }

    // 2. Step the simulator (if needed).
    if (!this->IsPaused()) {
      this->simulator->StepBy(this->timeStep);
    } else if (this->StepRequested()) {
      this->simulator->StepBy(this->CustomTimeStep());
    }

    // Remove any custom step request.
    this->SetStepRequested(false);

    {
      std::lock_guard<std::mutex> lock(this->mutex);

      // 3. Process outgoing messages (notifications).
      this->SendOutgoingMessages();

      // Do we have to exit?
      stayAlive = this->enabled;
    }

    // Stop the timer.
    auto stepEnd = std::chrono::steady_clock::now();

    // Wait for the remaining time of this step.
    auto stepElapsed = stepEnd - stepStart;
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int64_t>(this->timeStep * 1e6)) -
        stepElapsed);
  }
}

//////////////////////////////////////////////////
double SimulatorRunner::TimeStep() const { return this->timeStep; }

//////////////////////////////////////////////////
void SimulatorRunner::SetTimeStep(const double _timeStep) {
  this->timeStep = _timeStep;
}

//////////////////////////////////////////////////
bool SimulatorRunner::IsPaused() const { return this->paused; }

//////////////////////////////////////////////////
void SimulatorRunner::SetPaused(const bool _paused) { this->paused = _paused; }

//////////////////////////////////////////////////
void SimulatorRunner::ProcessIncomingMessages() {
  while (!this->incomingMsgs.empty()) {
    auto nextMsg = this->incomingMsgs.front();
    this->incomingMsgs.pop();

    // Process the message.
    switch (nextMsg.type()) {
      case ignition::msgs::SimulationInMessage::WORLDCONTROL:
        this->ProcessWorldControlMessage(nextMsg.world_control());
        break;

      case ignition::msgs::SimulationInMessage::ROBOTMODELREQUEST:
        this->ProcessRobotModelRequest(nextMsg.robot_model_request());
        break;

      default:
        throw std::runtime_error(
            "Unable to process msg of type: " +
            SimulationInMessage_SimMsgType_Name(nextMsg.type()));
        break;
    }
  }
}

//////////////////////////////////////////////////
void SimulatorRunner::SendOutgoingMessages() {
  while (!this->outgoingMsgs.empty()) {
    auto nextMsg = this->outgoingMsgs.front();
    this->outgoingMsgs.pop();

    // Send the message.
    this->notificationsPub.Publish(nextMsg);
  }
}

//////////////////////////////////////////////////
void SimulatorRunner::ProcessWorldControlMessage(
    const ignition::msgs::WorldControl& _msg) {
  if (_msg.has_pause()) {
    this->SetPaused(_msg.pause());
  } else if (_msg.has_step() && _msg.step()) {
    this->SetStepRequested(true);
    this->SetCustomTimeStep(this->TimeStep());
  } else if (_msg.has_multi_step() && _msg.multi_step() > 0u) {
    this->SetStepRequested(true);
    this->SetCustomTimeStep(this->TimeStep() * _msg.multi_step());
  } else {
    ignwarn << "Ignoring world control message" << std::endl;
  }
}

//////////////////////////////////////////////////
void SimulatorRunner::ProcessRobotModelRequest(
    const ignition::msgs::RobotModelRequest& _msg) {
  // Use the string on the robot model request as the topic name
  // where the robot model will be published
  auto robot_model = simulator->GetRobotModel();
  std::string topic_name = _msg.response_topic();

  auto pub = this->node.Advertise<ignition::msgs::Model_V>(topic_name);
  if (!pub) {
    ignerr << "Error advertising topic [" << topic_name << "]" << std::endl;
  }

  // Wait for 200 millis before attempting to
  // publish on the topic, it wont work otherwise
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  if(!pub.Publish(*robot_model)) {
    ignerr << "Error publishing message on topic [" << topic_name << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void SimulatorRunner::OnWorldControl(
    const ignition::msgs::WorldControl& request,
    ignition::msgs::Boolean& response, bool& result) {
  {
    // Queue the message.
    std::lock_guard<std::mutex> lock(this->mutex);
    ignition::msgs::SimulationInMessage input_message;
    input_message.set_type(ignition::msgs::SimulationInMessage::WORLDCONTROL);
    input_message.mutable_world_control()->CopyFrom(request);
    this->incomingMsgs.push(input_message);
  }
  result = true;
}

//////////////////////////////////////////////////
void SimulatorRunner::OnRobotModelRequest(
    const ignition::msgs::RobotModelRequest& request,
    ignition::msgs::Boolean& response, bool& result) {
  {
    // Queue the message.
    std::lock_guard<std::mutex> lock(this->mutex);
    ignition::msgs::SimulationInMessage input_message;
    input_message.set_type(
        ignition::msgs::SimulationInMessage::ROBOTMODELREQUEST);
    input_message.mutable_robot_model_request()->CopyFrom(request);
    this->incomingMsgs.push(input_message);
  }
  result = true;
}

//////////////////////////////////////////////////
bool SimulatorRunner::StepRequested() const { return this->stepRequested; }

//////////////////////////////////////////////////
void SimulatorRunner::SetStepRequested(const bool _stepRequested) {
  this->stepRequested = _stepRequested;
}

//////////////////////////////////////////////////
double SimulatorRunner::CustomTimeStep() const { return this->customTimeStep; }

//////////////////////////////////////////////////
void SimulatorRunner::SetCustomTimeStep(const double _timeStep) {
  this->customTimeStep = _timeStep;
}

}  // namespace backend
}  // namespace delphyne
