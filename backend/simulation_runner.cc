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

#include "backend/simulation_runner.h"

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <drake/common/unused.h>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace delphyne {
namespace backend {
namespace {

// \brief Flag to detect SIGINT or SIGTERM while the code is executing
// WaitForShutdown().
static bool g_shutdown = false;

// \brief Mutex to protect the boolean shutdown variable.
static std::mutex g_shutdown_mutex;

// \brief Condition variable to wakeup WaitForShutdown() and exit.
static std::condition_variable g_shutdown_cv;

// \brief Function executed when a SIGINT or SIGTERM signals are captured.
// \param[in] signal Signal received.
static void SignalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    {
      std::unique_lock<std::mutex> lk(g_shutdown_mutex);
      g_shutdown = true;
    }
    g_shutdown_cv.notify_all();
  }
}

}  // namespace

void WaitForShutdown() {
  // Installs a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::unique_lock<std::mutex> lk(g_shutdown_mutex);
  g_shutdown_cv.wait(lk, [] { return g_shutdown; });
}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
    double time_step, bool paused)
    : time_step_(time_step), simulator_(std::move(sim)), paused_(paused) {
  // Advertises the service for controlling the simulation.
  node_.Advertise(kControlService, &SimulatorRunner::OnWorldControl, this);

  // Advertise the topic for publishing notifications.
  notifications_pub_ =
      node_.Advertise<ignition::msgs::WorldControl>(kNotificationsTopic);

  // Advertise the service for receiving robot model requests from the frontend
  if (!node_.Advertise(kRobotRequestServiceName,
                       &SimulatorRunner::OnRobotModelRequest, this)) {
    ignerr << "Error advertising service [" << kRobotRequestServiceName << "]"
           << std::endl;
  }
  // Initializes the python machinery so we can invoke a python callback
  // function on each simulation step.
  Py_Initialize();
  PyEval_InitThreads();

  simulator_->Start();
}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
    double time_step)
    : SimulatorRunner(std::move(sim), time_step, false) {}

SimulatorRunner::~SimulatorRunner() {
  Stop();
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
}

void SimulatorRunner::Stop() {
  // Only do this if we are running the simulation.
  if (enabled_) {
    // Tells the main loop thread to terminate.
    enabled_ = false;
  }
}

void SimulatorRunner::AddStepCallback(std::function<void()> callable) {
  std::lock_guard<std::mutex> lock(mutex_);
  step_callbacks_.push_back(callable);
}

void SimulatorRunner::Start() {
  // The main loop is already running.
  if (enabled_) return;

  enabled_ = true;

  // Starts the thread that receives discovery information.
  main_thread_ = std::thread(&SimulatorRunner::Run, this);
}

void SimulatorRunner::Run() {
  while (enabled_) {
    RunSimulationStep();
  }
}

void SimulatorRunner::RunSimulationStep() {
  // Starts a timer to measure the time we spend doing tasks.
  auto step_start = std::chrono::steady_clock::now();

  // 1. Processes incoming messages (requests).
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ProcessIncomingMessages();
  }

  // 2. Steps the simulator (if needed).
  if (!paused_) {
    simulator_->StepBy(time_step_);
  } else if (step_requested_) {
    simulator_->StepBy(custom_time_step_);
  }

  // Removes any custom step request.
  step_requested_ = false;

  // A copy of the python callbacks so we can process them in a thread-safe
  // way
  std::vector<std::function<void()>> callbacks;

  // 3. Processes outgoing messages (notifications).
  {
    std::lock_guard<std::mutex> lock(mutex_);

    SendOutgoingMessages();

    // Makes a temporal copy of the python callbacks while we have the lock.
    callbacks = step_callbacks_;
  }

  // This if is here so that we only grab the python global interpreter lock
  // if there is at least one callback and the simulation is unpaused.
  if (callbacks.size() > 0 && !paused_) {
    // 1. Acquires the lock to the python interpreter.
    py::gil_scoped_acquire acquire;
    // 2. Performs the callbacks.
    for (std::function<void()> callback : callbacks) {
      callback();
    }
  }

  // Stops the timer.
  auto step_end = std::chrono::steady_clock::now();

  // Waits for the remaining time of this step.
  auto step_elapsed = step_end - step_start;
  std::this_thread::sleep_for(
      std::chrono::microseconds(static_cast<int64_t>(time_step_ * 1e6)) -
      step_elapsed);
}

void SimulatorRunner::ProcessIncomingMessages() {
  while (!incoming_msgs_.empty()) {
    const ignition::msgs::SimulationInMessage next_msg = incoming_msgs_.front();
    incoming_msgs_.pop();

    // Processes the message.
    switch (next_msg.type()) {
      case ignition::msgs::SimulationInMessage::WORLDCONTROL:
        ProcessWorldControlMessage(next_msg.world_control());
        break;

      case ignition::msgs::SimulationInMessage::ROBOTMODELREQUEST:
        this->ProcessRobotModelRequest(next_msg.robot_model_request());
        break;

      default:
        ignerr << "Unable to process msg of type: "
               << SimulationInMessage_SimMsgType_Name(next_msg.type())
               << std::endl;
        break;
    }
  }
}

void SimulatorRunner::SendOutgoingMessages() {
  while (!outgoing_msgs_.empty()) {
    const ignition::msgs::WorldControl next_msg = outgoing_msgs_.front();
    outgoing_msgs_.pop();

    // Sends the message.
    notifications_pub_.Publish(next_msg);
  }
}

void SimulatorRunner::ProcessWorldControlMessage(
    const ignition::msgs::WorldControl& msg) {
  if (msg.has_pause()) {
    paused_ = msg.pause();
  } else if (msg.has_step() && msg.step()) {
    step_requested_ = true;
    custom_time_step_ = msg.step();
  } else if (msg.has_multi_step() && msg.multi_step() > 0u) {
    step_requested_ = true;
    custom_time_step_ = time_step_ * msg.multi_step();
  } else {
    ignwarn << "Ignoring world control message" << std::endl;
  }
}

void SimulatorRunner::ProcessRobotModelRequest(
    const ignition::msgs::RobotModelRequest& msg) {
  // Sets the string from the robot model request as
  // the topic name where the robot model will be published
  auto robot_model = simulator_->GetRobotModel();
  std::string topic_name = msg.response_topic();

  node_.Request(topic_name, *robot_model);
}

void SimulatorRunner::OnWorldControl(
    const ignition::msgs::WorldControl& request,
    ignition::msgs::Boolean& response, bool& result) {
  // Fill the new message.
  ignition::msgs::SimulationInMessage input_message;
  input_message.set_type(ignition::msgs::SimulationInMessage::WORLDCONTROL);
  input_message.mutable_world_control()->CopyFrom(request);

  {
    // Queue the message.
    std::lock_guard<std::mutex> lock(mutex_);
    incoming_msgs_.push(input_message);
  }
  result = true;
}

void SimulatorRunner::OnRobotModelRequest(
    const ignition::msgs::RobotModelRequest& request,
    ignition::msgs::Boolean& response, bool& result) {
  // Fill the new message.
  ignition::msgs::SimulationInMessage input_message;
  input_message.set_type(
      ignition::msgs::SimulationInMessage::ROBOTMODELREQUEST);
  input_message.mutable_robot_model_request()->CopyFrom(request);

  {
    // Queue the message.
    std::lock_guard<std::mutex> lock(mutex_);
    incoming_msgs_.push(input_message);
  }
  result = true;
}

}  // namespace backend
}  // namespace delphyne
