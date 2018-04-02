// Copyright 2017 Toyota Research Institute

#include "backend/simulation_runner.h"

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <limits>
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

// @brief Flag to detect SIGINT or SIGTERM while the code is executing
// WaitForShutdown().
static bool g_shutdown = false;

// @brief Mutex to protect the boolean shutdown variable.
static std::mutex g_shutdown_mutex;

// @brief Condition variable to wakeup WaitForShutdown() and exit.
static std::condition_variable g_shutdown_cv;

// @brief Function executed when a SIGINT or SIGTERM signals are captured.
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
    double time_step, double realtime_rate, bool paused)
    : time_step_(time_step), simulator_(std::move(sim)), paused_(paused) {
  // Advertises the service for controlling the simulation.
  node_.Advertise(kControlService, &SimulatorRunner::OnWorldControl, this);

  // Advertise the topic for publishing notifications.
  notifications_pub_ =
      node_.Advertise<ignition::msgs::WorldControl>(kNotificationsTopic);

  // Advertise the topic for publishing world stats.
  world_stats_pub_ =
      node_.Advertise<ignition::msgs::WorldStatistics>(kWorldStatsTopic);

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

  simulator_->Start(realtime_rate);
}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
    double time_step, bool paused)
    : SimulatorRunner(std::move(sim), time_step, 1.0, paused) {}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
    double time_step, double realtime_rate)
    : SimulatorRunner(std::move(sim), time_step, realtime_rate, false) {}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
    double time_step)
    : SimulatorRunner(std::move(sim), time_step, 1.0, false) {}

SimulatorRunner::~SimulatorRunner() {
  // In case the simulation thread was running and the client code didn't
  // call the Stop() method.
  enabled_ = false;
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
}

void SimulatorRunner::Stop() {
  DELPHYNE_DEMAND(enabled_);
  enabled_ = false;
}

bool SimulatorRunner::IsPaused() const { return paused_; }

bool SimulatorRunner::IsRunning() const { return enabled_; }

void SimulatorRunner::RequestMultiStep(unsigned int num_steps) {
  DELPHYNE_DEMAND(enabled_);
  DELPHYNE_DEMAND(paused_);

  // Ignore the request if we're already processing a previous multi-step.
  if (custom_num_steps_ > 0) {
    igndbg << "Ignoring MultiStep request (a previous multi-step is ongoing."
           << std::endl;
    return;
  }

  igndbg << "Resetting simulation statistics." << std::endl;
  simulator_->ResetStatistics();
  custom_num_steps_ = num_steps;
}

void SimulatorRunner::Unpause() {
  // TODO(apojomovsky): We should revisit this once we get feedback on
  // https://github.com/RobotLocomotion/drake/issues/8090
  if (paused_) {
    igndbg << "Resetting simulation statistics." << std::endl;
    simulator_->ResetStatistics();
    paused_ = false;
    custom_num_steps_ = 0u;
  }
}

void SimulatorRunner::Pause() { paused_ = true; }

void SimulatorRunner::AddStepCallback(std::function<void()> callable) {
  std::lock_guard<std::mutex> lock(mutex_);
  step_callbacks_.push_back(callable);
}

void SimulatorRunner::Start() {
  this->RunAsyncFor(std::numeric_limits<double>::infinity(), [] {});
}

void SimulatorRunner::RunAsyncFor(double duration,
                                  std::function<void()> callback) {
  DELPHYNE_DEMAND(!enabled_);
  enabled_ = true;
  main_thread_ = std::thread([this, duration, callback]() {
    this->RunSimulationLoopFor(duration, callback);
  });
}

void SimulatorRunner::RunSyncFor(double duration) {
  DELPHYNE_DEMAND(!enabled_);
  enabled_ = true;
  this->RunSimulationLoopFor(duration, [] {});
}

void SimulatorRunner::RunSimulationLoopFor(double duration,
                                           std::function<void()> callback) {
  DELPHYNE_DEMAND(duration >= 0);
  DELPHYNE_DEMAND(callback != nullptr);

  const double sim_end = simulator_->get_current_simulation_time() + duration;

  while (enabled_ && (simulator_->get_current_simulation_time() < sim_end)) {
    RunSimulationStep();
  }

  enabled_ = false;

  callback();
}

void SimulatorRunner::RunSimulationStep() {
  // 1. Processes incoming messages (requests).
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ProcessIncomingMessages();
  }

  // 2. Steps the simulator (if needed). Note that the simulator will sleep
  // here if needed to adjust to the real-time rate.
  if (!paused_) {
    simulator_->StepBy(time_step_);
  } else if (custom_num_steps_ > 0) {
    simulator_->StepBy(time_step_);
    custom_num_steps_--;
  }

  // A copy of the python callbacks so we can process them in a thread-safe
  // way
  std::vector<std::function<void()>> callbacks;

  // 3. Processes outgoing messages (notifications) and send world stats.
  {
    std::lock_guard<std::mutex> lock(mutex_);

    SendOutgoingMessages();

    SendWorldStats();

    // Makes a temporal copy of the python callbacks while we have the lock.
    callbacks = step_callbacks_;
  }

  // This if is here so that we only grab the python global
  // interpreter lock if there is at least one callback.
  if (callbacks.size() > 0) {
    // 1. Acquires the lock to the python interpreter.
    py::gil_scoped_acquire acquire;
    // 2. Performs the callbacks.
    for (std::function<void()> callback : callbacks) {
      callback();
    }
  }
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

void SimulatorRunner::SendWorldStats() {
  // Check if it's time to update the world stats.
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = now - last_world_stats_update_;
  if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() <
      kWorldStatsPeriodMs_) {
    return;
  }

  // It's time to update the world stats!
  last_world_stats_update_ = now;

  ignition::msgs::WorldStatistics msg;
  msg.set_paused(paused_);

  // TODO(caguero): Fill other fields when relevant.

  // Sends the message.
  world_stats_pub_.Publish(msg);
}

void SimulatorRunner::ProcessWorldControlMessage(
    const ignition::msgs::WorldControl& msg) {
  if (msg.has_pause()) {
    if (msg.pause()) {
      Pause();
    } else {
      Unpause();
    }
  } else if (msg.has_step() && msg.step()) {
    RequestMultiStep(1u);
  } else if (msg.has_multi_step() && msg.multi_step() > 0u) {
    RequestMultiStep(msg.multi_step());
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

bool SimulatorRunner::OnWorldControl(
    const ignition::msgs::WorldControl& request,
    ignition::msgs::Boolean& response) {
  // Fill the new message.
  ignition::msgs::SimulationInMessage input_message;
  input_message.set_type(ignition::msgs::SimulationInMessage::WORLDCONTROL);
  input_message.mutable_world_control()->CopyFrom(request);

  {
    // Queue the message.
    std::lock_guard<std::mutex> lock(mutex_);
    incoming_msgs_.push(input_message);
  }
  return true;
}

bool SimulatorRunner::OnRobotModelRequest(
    const ignition::msgs::RobotModelRequest& request,
    ignition::msgs::Boolean& response) {
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
  return true;
}

}  // namespace backend
}  // namespace delphyne
