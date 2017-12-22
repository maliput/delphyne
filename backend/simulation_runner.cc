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
#include <pybind11/pybind11.h>

#include "backend/simulation_runner.h"

namespace py = pybind11;

namespace delphyne {
namespace backend {
/// \brief Flag to detect SIGINT or SIGTERM while the code is executing
/// WaitForShutdown().
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
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::unique_lock<std::mutex> lk(g_shutdown_mutex);
  g_shutdown_cv.wait(lk, [] { return g_shutdown; });
}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<drake::automotive::AutomotiveSimulator<double>> sim,
    const double time_step)
    : time_step_(time_step), simulator_(std::move(sim)) {
  // Advertise the service for controlling the simulation.
  this->node_.Advertise(this->kControlService,
                        &SimulatorRunner::OnSimulationInMessage, this);

  // Advertise the topic for publishing notifications.
  this->notifications_pub_ =
      this->node_.Advertise<ignition::msgs::WorldControl>(
          this->kNotificationsTopic);

  // Initialize the python machinery so we can invoke a python callback
  // function on each simulation step.
  Py_Initialize();
  PyEval_InitThreads();

  this->simulator_->Start();
}

SimulatorRunner::~SimulatorRunner() {
  Stop();
  if (this->main_thread_.joinable()) {
    this->main_thread_.join();
  }
}

void SimulatorRunner::Stop() {
  // Only do this if we are running the simulation
  if (this->enabled_) {
    // Tell the main loop thread to terminate.
    this->enabled_ = false;
  }
}

void SimulatorRunner::AddStepCallback(std::function<void()> callable) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  step_callbacks_.push_back(callable);
}

void SimulatorRunner::Start() {
  // The main loop is already running.
  if (this->enabled_) return;

  this->enabled_ = true;

  // Start the thread that receives discovery information.
  this->main_thread_ = std::thread(&SimulatorRunner::Run, this);
}

void SimulatorRunner::Run() {
  while (this->enabled_) {
    // Start a timer to measure the time we spend doing tasks.
    auto step_start = std::chrono::steady_clock::now();

    // A copy of the python callbacks so we can process them in a thread-safe
    // way
    std::vector<std::function<void()>> callbacks;

    // 1. Process incoming messages (requests).
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->ProcessIncomingMessages();
    }

    // 2. Step the simulator (if needed).
    if (!this->IsPaused()) {
      this->simulator_->StepBy(this->time_step_);
    } else if (this->StepRequested()) {
      this->simulator_->StepBy(this->CustomTimeStep());
    }

    // Remove any custom step request.
    this->SetStepRequested(false);

    {
      std::lock_guard<std::mutex> lock(this->mutex_);

      // 3. Process outgoing messages (notifications).
      this->SendOutgoingMessages();

      // Make a temporal copy of the python callbacks while we have the lock.
      callbacks = step_callbacks_;
    }

    // This if is here so that we only grab the python global interpreter lock
    // if there is at least one callback.
    if (callbacks.size() > 0) {
      // 1. Acquire the lock to the python interpreter
      py::gil_scoped_acquire acquire;
      // 2. Perform the callbacks
      for (std::function<void()> callback : callbacks) {
        callback();
      }
    }

    // Stop the timer.
    auto step_end = std::chrono::steady_clock::now();

    // Wait for the remaining time of this step.
    auto step_elapsed = step_end - step_start;
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(
                                    this->time_step_ * 1e6)) -
                                step_elapsed);
  }
}

double SimulatorRunner::TimeStep() const { return this->time_step_; }

void SimulatorRunner::SetTimeStep(const double time_step) {
  this->time_step_ = time_step;
}

bool SimulatorRunner::IsPaused() const { return this->paused_; }

void SimulatorRunner::SetPaused(const bool paused) { this->paused_ = paused; }

void SimulatorRunner::ProcessIncomingMessages() {
  while (!this->incoming_msgs_.empty()) {
    auto next_msg = this->incoming_msgs_.front();
    this->incoming_msgs_.pop();

    // Process the message.
    switch (next_msg.type()) {
      case ignition::msgs::SimulationInMessage::WORLDCONTROL:
        this->ProcessWorldControlMessage(next_msg.world_control());
        break;
      default:
        throw std::runtime_error(
            "Unable to process msg of type: " +
            SimulationInMessage_SimMsgType_Name(next_msg.type()));
        break;
    }
  }
}

void SimulatorRunner::SendOutgoingMessages() {
  while (!this->outgoing_msgs_.empty()) {
    auto next_msg = this->outgoing_msgs_.front();
    this->outgoing_msgs_.pop();

    // Send the message.
    this->notifications_pub_.Publish(next_msg);
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

void SimulatorRunner::OnSimulationInMessage(
    const ignition::msgs::SimulationInMessage& request,
    ignition::msgs::Boolean& response, bool& result) {
  {
    // Just queue the message.
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->incoming_msgs_.push(request);
  }

  result = true;
}

bool SimulatorRunner::StepRequested() const { return this->step_requested_; }

void SimulatorRunner::SetStepRequested(const bool step_requested) {
  this->step_requested_ = step_requested;
}

double SimulatorRunner::CustomTimeStep() const {
  return this->custom_time_step_;
}

void SimulatorRunner::SetCustomTimeStep(const double time_step) {
  this->custom_time_step_ = time_step;
}

}  // namespace backend
}  // namespace delphyne
