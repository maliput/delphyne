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
    std::unique_ptr<drake::automotive::AutomotiveSimulator<double>> _sim,
    const double _timeStep)
    : timeStep(_timeStep), simulator(std::move(_sim)) {
  // Advertise the service for controlling the simulation.
  this->node.Advertise(this->kControlService,
                       &SimulatorRunner::OnSimulationInMessage, this);

  // Advertise the topic for publishing notifications.
  this->notificationsPub = this->node.Advertise<ignition::msgs::WorldControl>(
      this->kNotificationsTopic);

  // Initialize the python machinery so we can invoke a python callback
  // function on each simulation step.
  Py_Initialize();
  PyEval_InitThreads();

  this->simulator->Start();
}

//////////////////////////////////////////////////
SimulatorRunner::~SimulatorRunner() {
  Stop();
  if (this->mainThread.joinable()) {
    this->mainThread.join();
  }
}

//////////////////////////////////////////////////
void SimulatorRunner::Stop() {
  // Only do this if we are running the simulation
  if (this->enabled) {
    // Tell the main loop thread to terminate.
    std::lock_guard<std::mutex> lock(this->mutex);
    this->enabled = false;
  }
}

//////////////////////////////////////////////////
void SimulatorRunner::AddStepCallback(PyObject* callable) {
  step_callbacks.push_back(callable);
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

    // This if is here so that we only grab the python global interpreter lock
    // if there is at least one callback.
    if (!step_callbacks.empty()) {
      // 1. Acquire the lock to the python interpreter
      auto thread_handle = PyGILState_Ensure();
      // 2. Perform the callbacks
      for (auto callback : step_callbacks) {
        boost::python::call<void>(callback);
      }
      // 3. Release the lock
      PyGILState_Release(thread_handle);
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
void SimulatorRunner::OnSimulationInMessage(
    const ignition::msgs::SimulationInMessage& _req,
    ignition::msgs::Boolean& _rep, bool& _result) {
  {
    // Just queue the message.
    std::lock_guard<std::mutex> lock(this->mutex);
    this->incomingMsgs.push(_req);
  }

  _result = true;
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
