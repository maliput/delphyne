// Copyright 2017 Toyota Research Institute

#include "backend/simulation_runner.h"

#include <dirent.h>
#include <libgen.h>

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include <drake/common/unused.h>

#include <ignition/common/Console.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <pybind11/pybind11.h>

#include "common/filesystem.h"

namespace delphyne {
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
    std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
    double time_step, double realtime_rate, bool paused, bool log,
    std::string logfile_name)
    : time_step_(time_step),
      simulator_(std::move(sim)),
      realtime_rate_(realtime_rate),
      paused_(paused) {
  DELPHYNE_VALIDATE(realtime_rate >= 0.0, std::invalid_argument,
                    "Realtime rate must be >= 0.0");

  // Advertises the service for controlling the simulation.
  node_.Advertise(kControlService, &SimulatorRunner::OnWorldControl, this);

  // Advertise the topic for publishing notifications.
  notifications_pub_ =
      node_.Advertise<ignition::msgs::WorldControl>(kNotificationsTopic);

  // Advertise the topic for publishing world stats.
  world_stats_pub_ =
      node_.Advertise<ignition::msgs::WorldStatistics>(kWorldStatsTopic);

  // Advertise the service for receiving scene requests from the frontend
  if (!node_.Advertise(kSceneRequestServiceName,
                       &SimulatorRunner::OnSceneRequest, this)) {
    ignerr << "Error advertising service [" << kSceneRequestServiceName << "]"
           << std::endl;
  }

  // The get_internals() function initializes the `internals.tstate` for
  // subsequent `gil_scoped_acquire` calls. If the gil_scoped_acquire is
  // attempted to be called before having initialized the internals.tstate,
  // it would end up into a segmentation fault. This initialization should
  // only occur if the SimulatorRunner class is instantiated from within a
  // python script, which is checked in the if statement.
  // In comment below, a similar approach to the one used here is suggested:
  // https://github.com/pybind/pybind11/issues/1360#issuecomment-385988887
  if (Py_IsInitialized()) {
    pybind11::detail::get_internals();
  }

  // Tell the simulator to run steps as fast as possible, as are handling the
  // sleep (if required) between steps.
  simulator_->Start(0.);

  if (log) {
    StartLogging(logfile_name);
  }
}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
    double time_step, bool paused, bool log)
    : SimulatorRunner(std::move(sim), time_step, 1.0, paused, log, "") {}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
    double time_step, bool paused, bool log, std::string logfile_name)
    : SimulatorRunner(std::move(sim), time_step, 1.0, paused, log,
                      logfile_name) {}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
    double time_step, double realtime_rate)
    : SimulatorRunner(std::move(sim), time_step, realtime_rate, false, false,
                      "") {}

SimulatorRunner::SimulatorRunner(
    std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
    double time_step)
    : SimulatorRunner(std::move(sim), time_step, 1.0, false, false, "") {}

SimulatorRunner::~SimulatorRunner() {
  // In case the simulation thread was running and the client code didn't
  // call the Stop() method.
  interactive_loop_running_ = false;
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
  StopLogging();
}

void SimulatorRunner::PauseSimulation() {
  DELPHYNE_VALIDATE(!paused_, std::runtime_error,
                    "Cannot pause already paused simulation");
  paused_ = true;
}

void SimulatorRunner::UnpauseSimulation() {
  DELPHYNE_VALIDATE(paused_, std::runtime_error,
                    "Cannot unpause already running simulation");

  // If there are any pending step requests, erase them
  steps_requested_ = 0;

  SetupNewRunStats();

  paused_ = false;
}

void SimulatorRunner::AddStepCallback(std::function<void()> callable) {
  std::lock_guard<std::mutex> lock(mutex_);
  step_callbacks_.push_back(callable);
}

void SimulatorRunner::AddCollisionCallback(CollisionCallback callable) {
  std::lock_guard<std::mutex> lock(mutex_);
  collision_callbacks_.push_back(callable);
}

void SimulatorRunner::Start() {
  this->RunAsyncFor(std::numeric_limits<double>::infinity(), [] {});
}

void SimulatorRunner::Stop() {
  DELPHYNE_VALIDATE(interactive_loop_running_, std::runtime_error,
                    "Cannot stop a simulation that is not running");

  // If there are any pending step requests, erase them
  steps_requested_ = 0;

  interactive_loop_running_ = false;

  StopLogging();
}

void SimulatorRunner::RunAsyncFor(double duration,
                                  std::function<void()> callback) {
  DELPHYNE_VALIDATE(!interactive_loop_running_, std::runtime_error,
                    "Cannot run a simulation that is already running");
  interactive_loop_running_ = true;
  main_thread_ = std::thread([this, duration, callback]() {
    this->RunInteractiveSimulationLoopFor(duration, callback);
  });
}

void SimulatorRunner::RunSyncFor(double duration) {
  DELPHYNE_VALIDATE(!interactive_loop_running_, std::runtime_error,
                    "Cannot run a simulation that is already running");
  interactive_loop_running_ = true;
  this->RunInteractiveSimulationLoopFor(duration, [] {});
}

void SimulatorRunner::RunInteractiveSimulationLoopFor(
    double duration, std::function<void()> callback) {
  DELPHYNE_VALIDATE(duration >= 0.0, std::invalid_argument,
                    "Duration must be >= 0.0");
  DELPHYNE_VALIDATE(callback != nullptr, std::invalid_argument,
                    "Callback must not be null");

  SetupNewRunStats();

  const double sim_end = simulator_->GetCurrentSimulationTime() + duration;

  while (interactive_loop_running_ &&
         (simulator_->GetCurrentSimulationTime() < sim_end)) {
    RunInteractiveSimulationLoopStep();
  }

  interactive_loop_running_ = false;

  callback();
}

void SimulatorRunner::RunInteractiveSimulationLoopStep() {
  // 1. Processes incoming messages (requests).
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ProcessIncomingMessages();
  }

  // 2. Steps the simulator (if needed). Note that the simulator will sleep
  // here if needed to adjust to the real-time rate.
  if (!paused_) {
    StepSimulationBy(time_step_);
  } else if (steps_requested_ > 0) {
    StepSimulationBy(time_step_);
    steps_requested_--;
  }

  // A copy of the python callbacks so we can process them in a thread-safe
  // way
  std::vector<std::function<void()>> step_callbacks;
  std::vector<CollisionCallback> collision_callbacks;

  // 3. Processes outgoing messages (notifications) and send world stats.
  {
    std::lock_guard<std::mutex> lock(mutex_);

    SendOutgoingMessages();

    SendWorldStats();

    // Makes a temporal copy of the python callbacks while we have the lock.
    step_callbacks = step_callbacks_;
    collision_callbacks = collision_callbacks_;
  }

  // This if is here so that we only grab the python global
  // interpreter lock if there is at least one callback.
  if (!step_callbacks.empty()) {
    // 1. Acquires the lock to the python interpreter.
    pybind11::gil_scoped_acquire acquire;
    // 2. Performs the callbacks.
    for (std::function<void()> callback : step_callbacks) {
      callback();
    }
  }

  // Computes collisions iff collisions are enabled and simulation
  // is not paused.
  if (collisions_enabled_ && !paused_) {
    // Computes collisions between agents.
    const std::vector<
        std::pair<delphyne::AgentBase<double>*, delphyne::AgentBase<double>*>>
        agents_in_collision = simulator_->GetCollisions();
    if (!agents_in_collision.empty()) {
      // Pauses simulation if necessary.
      PauseSimulation();
      // Calls all registered collision callbacks, if any.
      if (!collision_callbacks.empty()) {
        // 1. Acquires the lock to the python interpreter.
        pybind11::gil_scoped_acquire acquire;
        // 2. Calls all collision callbacks.
        for (CollisionCallback callback : collision_callbacks) {
          callback(agents_in_collision);
        }
      }
    }
  }
}

void SimulatorRunner::StepSimulationBy(double time_step) {
  simulator_->StepBy(time_step);

  stats_.StepExecuted(simulator_->GetCurrentSimulationTime());

  // Return if running at full speed
  if (realtime_rate_ == 0) {
    return;
  }

  const TimePoint expected_realtime = stats_.CurrentStepExpectedRealtimeEnd();

  if (expected_realtime > RealtimeClock::now()) {
    std::this_thread::sleep_until(expected_realtime);
  }
}

void SimulatorRunner::SetupNewRunStats() {
  stats_.NewRunStartingAt(simulator_->GetCurrentSimulationTime(),
                          realtime_rate_);
}

void SimulatorRunner::RequestSimulationStepExecution(unsigned int steps) {
  DELPHYNE_VALIDATE(steps > 0, std::invalid_argument, "Steps must be > 0");
  DELPHYNE_VALIDATE(interactive_loop_running_, std::runtime_error,
                    "Cannot step a simulation that is not yet running");
  DELPHYNE_VALIDATE(paused_, std::runtime_error,
                    "Cannot step a simulation that is not paused");

  SetupNewRunStats();

  steps_requested_ = steps;
}

void SimulatorRunner::SetRealtimeRate(double realtime_rate) {
  DELPHYNE_VALIDATE(realtime_rate >= 0.0, std::invalid_argument,
                    "Realtime rate must be >= 0.0");
  realtime_rate_ = realtime_rate;
  SetupNewRunStats();
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

      case ignition::msgs::SimulationInMessage::SCENEREQUEST:
        this->ProcessSceneRequest(next_msg.scene_request());
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
  msg.set_real_time_factor(stats_.get_current_realtime_rate());

  // TODO(caguero): Fill other fields when relevant.

  // Sends the message.
  world_stats_pub_.Publish(msg);
}

void SimulatorRunner::ProcessWorldControlMessage(
    const ignition::msgs::WorldControl& msg) {
  if (msg.has_pause()) {
    if (msg.pause()) {
      PauseSimulation();
    } else {
      UnpauseSimulation();
    }
  } else if (msg.has_step() && msg.step()) {
    RequestSimulationStepExecution(1u);
  } else if (msg.has_multi_step() && msg.multi_step() > 0u) {
    RequestSimulationStepExecution(msg.multi_step());
  } else {
    ignwarn << "Ignoring world control message" << std::endl;
  }
}

void SimulatorRunner::ProcessSceneRequest(
    const ignition::msgs::SceneRequest& msg) {
  // Sets the string from the scene request as
  // the topic name where the scene will be published
  const std::unique_ptr<ignition::msgs::Scene> scene = simulator_->GetScene();
  const std::string topic_name = msg.response_topic();

  node_.Request(topic_name, *scene);
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

bool SimulatorRunner::OnSceneRequest(
    const ignition::msgs::SceneRequest& request,
    ignition::msgs::Boolean& response) {
  // Fill the new message.
  ignition::msgs::SimulationInMessage input_message;
  input_message.set_type(ignition::msgs::SimulationInMessage::SCENEREQUEST);
  input_message.mutable_scene_request()->CopyFrom(request);

  {
    // Queue the message.
    std::lock_guard<std::mutex> lock(mutex_);
    incoming_msgs_.push(input_message);
  }
  return true;
}

namespace {

// @brief Creates the filename under which a log will be saved.
// @return The logfile name with extension.
std::string GenerateDefaultLogFilename() {
  std::stringstream filename{};
  // Get environmental variable.
  const char* delphyne_logs_prefix =
      std::getenv("DELPHYNE_LOGS_PREFIX");
  if (delphyne_logs_prefix == NULL) {
    igndbg << "Unable to get DELPHYNE_LOGS_PREFIX"
           << " environment variable." << std::endl;
  } else {
    // Appends the prefix taken from the env var.
    filename << delphyne_logs_prefix << "-";
  }
  // Constructs a timestamp log file name.
  std::time_t now = std::time(nullptr);
  std::tm tm = *std::localtime(&now);
  filename << std::put_time(&tm, "%FT%H%M%S%z");
  // Adds extension.
  filename << ".log";
  return filename.str();
}

}  // namespace

void SimulatorRunner::StartLogging() {
  StartLogging(GenerateDefaultLogFilename());
}

void SimulatorRunner::StartLogging(const std::string& filename) {
  std::string sanitized_filename = filename;
  if (!IsValidFilepath(filename)) {
    ignerr << "The given log filename is ill-formed. "
           << "Logging to default location." << std::endl;
    sanitized_filename = GenerateDefaultLogFilename();
  }
  std::unique_ptr<ignition::msgs::Scene> scene = simulator_->GetScene();
  logger_.Start(sanitized_filename);
  logger_.CaptureMeshes(*scene);
}

void SimulatorRunner::StopLogging() {
  if (logger_.is_logging()) {
    logger_.Stop();
  }
}

std::string SimulatorRunner::GetLogFilename() const {
  return logger_.logpath();
}

}  // namespace delphyne
