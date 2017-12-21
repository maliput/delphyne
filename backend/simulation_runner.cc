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

/// \brief Flag to detect SIGINT or SIGTERM while the code is executing.
static bool g_shutdown = false;

/// \brief Mutex to protect the boolean shutdown variable.
static std::mutex g_shutdown_mutex;

/// \brief Condition variable to wakeup WaitForShutdown() and exit.
static std::condition_variable g_shutdown_cv;

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

void WaitForShutdown() {
  // Installs a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::unique_lock<std::mutex> lk(g_shutdown_mutex);
  g_shutdown_cv.wait(lk, [] { return g_shutdown; });
}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
    double timestep)
    : timestep_(timestep), simulator_(std::move(sim)) {
  // Advertises the service for controlling the simulation.
  this->node_.Advertise(this->kControlService, &SimulatorRunner::OnWorldControl,
                        this);

  // Advertises the topic for publishing notifications.
  this->notifications_pub_ =
      this->node_.Advertise<ignition::msgs::WorldControl>(
          this->kNotificationsTopic);

  // Advertises the service for receiving robot model requests from the frontend.
  if (!this->node_.Advertise(kRobotRequestServiceName,
                             &SimulatorRunner::OnRobotModelRequest, this)) {
    ignerr << "Error advertising service [" << kRobotRequestServiceName << "]"
           << std::endl;
  }

  this->simulator_->Start();
}

SimulatorRunner::~SimulatorRunner() {
  {
    // Tells the main loop thread to terminate.
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->enabled_ = false;
  }
  if (this->main_thread_.joinable()) {
    this->main_thread_.join();
  }
}

void SimulatorRunner::Start() {
  // The main loop is already running.
  if (this->enabled_) return;

  this->enabled_ = true;

  // Starts the thread that receives discovery information.
  this->main_thread_ = std::thread(&SimulatorRunner::Run, this);
}

void SimulatorRunner::Run() {
  bool stayAlive = true;
  while (stayAlive) {
    // Starts a timer to measure the time we spend doing tasks.
    auto stepStart = std::chrono::steady_clock::now();

    // 1. Processes incoming messages (requests).
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->ProcessIncomingMessages();
    }

    // 2. Steps the simulator (if needed).
    if (!this->IsPaused()) {
      this->simulator_->StepBy(this->timestep_);
    } else if (this->StepRequested()) {
      this->simulator_->StepBy(this->CustomTimeStep());
    }

    // Removes any custom step request.
    this->SetStepRequested(false);

    {
      std::lock_guard<std::mutex> lock(this->mutex_);

      // 3. Processes outgoing messages (notifications).
      this->SendOutgoingMessages();

      // Do we have to exit?.
      stayAlive = this->enabled_;
    }

    // Stops the timer.
    auto stepEnd = std::chrono::steady_clock::now();

    // Waits for the remaining time of this step.
    auto stepElapsed = stepEnd - stepStart;
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int64_t>(this->timestep_ * 1e6)) -
        stepElapsed);
  }
}

double SimulatorRunner::TimeStep() const { return this->timestep_; }

void SimulatorRunner::SetTimeStep(const double timestep) {
  this->timestep_ = timestep;
}

bool SimulatorRunner::IsPaused() const { return this->paused_; }

void SimulatorRunner::SetPaused(const bool paused) { this->paused_ = paused; }

void SimulatorRunner::ProcessIncomingMessages() {
  while (!this->incoming_msgs_.empty()) {
    auto nextMsg = this->incoming_msgs_.front();
    this->incoming_msgs_.pop();

    // Processes the message.
    switch (nextMsg.type()) {
      case ignition::msgs::SimulationInMessage::WORLDCONTROL:
        this->ProcessWorldControlMessage(nextMsg.world_control());
        break;

      case ignition::msgs::SimulationInMessage::ROBOTMODELREQUEST:
        this->ProcessRobotModelRequest(nextMsg.robot_model_request());
        break;

      default:
        ignerr << "Unable to process msg of type: "
               << SimulationInMessage_SimMsgType_Name(nextMsg.type())
               << std::endl;
        break;
    }
  }
}

void SimulatorRunner::SendOutgoingMessages() {
  while (!this->outgoing_msgs_.empty()) {
    auto nextMsg = this->outgoing_msgs_.front();
    this->outgoing_msgs_.pop();

    // Sends the message.
    this->notifications_pub_.Publish(nextMsg);
  }
}

void SimulatorRunner::ProcessWorldControlMessage(
    const ignition::msgs::WorldControl& msg) {
  if (msg.has_pause()) {
    this->SetPaused(msg.pause());
  } else if (msg.has_step() && msg.step()) {
    this->SetStepRequested(true);
    this->SetCustomTimeStep(this->TimeStep());
  } else if (msg.has_multi_step() && msg.multi_step() > 0u) {
    this->SetStepRequested(true);
    this->SetCustomTimeStep(this->TimeStep() * msg.multi_step());
  } else {
    ignwarn << "Ignoring world control message" << std::endl;
  }
}

void SimulatorRunner::ProcessRobotModelRequest(
    const ignition::msgs::RobotModelRequest& msg) {
  // Sets the string from the robot model request as
  // the topic name where the robot model will be published.
  auto robot_model = simulator_->GetRobotModel();
  std::string topic_name = msg.response_topic();

  node_.Request(topic_name, *robot_model);
}

void SimulatorRunner::OnWorldControl(
    const ignition::msgs::WorldControl& request,
    ignition::msgs::Boolean& response, bool& result) {
  // Fills the new message.
  ignition::msgs::SimulationInMessage input_message;
  input_message.set_type(ignition::msgs::SimulationInMessage::WORLDCONTROL);
  input_message.mutable_world_control()->CopyFrom(request);

  {
    // Queues the message.
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->incoming_msgs_.push(input_message);
  }
  result = true;
}

void SimulatorRunner::OnRobotModelRequest(
    const ignition::msgs::RobotModelRequest& request,
    ignition::msgs::Boolean& response, bool& result) {
  // Fills the new message.
  ignition::msgs::SimulationInMessage input_message;
  input_message.set_type(
      ignition::msgs::SimulationInMessage::ROBOTMODELREQUEST);
  input_message.mutable_robot_model_request()->CopyFrom(request);

  {
    // Queues the message.
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->incoming_msgs_.push(input_message);
  }
  result = true;
}

bool SimulatorRunner::StepRequested() const { return this->step_requested_; }

void SimulatorRunner::SetStepRequested(const bool step_requested) {
  this->step_requested_ = step_requested;
}

double SimulatorRunner::CustomTimeStep() const {
  return this->custom_timestep_;
}

void SimulatorRunner::SetCustomTimeStep(const double timestep) {
  this->custom_timestep_ = timestep;
}

}  // namespace backend
}  // namespace delphyne
