// Copyright 2017 Toyota Research Institute

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/log/Recorder.hh>

#include "backend/automotive_simulator.h"
#include "backend/interactive_simulation_stats.h"
#include "delphyne/macros.h"
#include "delphyne/protobuf/scene_request.pb.h"
#include "delphyne/protobuf/simulation_in_message.pb.h"

namespace delphyne {

using delphyne::SimulationRunStats;

/// @brief Blocks the current thread until a SIGINT or SIGTERM is received.
/// Note that this function registers a signal handler. Do not use this
/// function if you want to manage yourself SIGINT/SIGTERM.
void WaitForShutdown();

/// @brief A wrapper to execute the Drake simulator as a back-end.
/// This class exposes some of the functionality of the simulator via Ignition
/// Transport services (e.g.: play/pause/step the simulation). At the same time,
/// it has to periodically advance the simulation a certain amount of time.
/// Finally, it notifies any given state changes to the outside world via
/// Ignition Transport messages. All these operations occur in a main loop with
/// the following structure:
///
/// * Process incoming messages: All incoming requests are queued
///   asynchronously when received and in this step we process all of them.
///   This request might involve flag the simulation as paused, insert a new
///   model, remove an existing model, etc.
///
/// * Step the simulation: We advance the simulation a certain amount of time.
///   The amount of time could be the default time step (if we are in play
///   mode), a custom time step (if we are externally stepping) or no time at
///   all if the simulation is paused.
///
/// * Notify changes in the simulation. The simulation will send messages over a
///   a well-known topic every time that a relevant simulation state changed
///   occur. E.g.: When the simulation is paused or a model is removed. These
///   notifications should be used by all the Visualizer instances to update its
///   state/widgets in order to keep them in sync with the back-end.
///
/// * Provides a mechanism to register callbacks to be triggered on each
///   simulation step. Since this was originally conceived to be used with the
///   Python bindings and plain typedef-based functions are not supported
///   (see http://www.boost.org/doc/libs/1_65_1/libs/python/doc/html/faq.html)
///   we need to pass a PyObject as an argument.
///
/// Finally, it is important to clarify how the Python-C++ callback mechanism
/// is implemented, so we can understand a possible deadlock. The key
/// elements here are:
///   - A typical main program will create a SimulatorRunner instance,
///   configure it and call Start(). Let's call the thread the program runs on
///   the MainThread.
///   - Calling Start() will in turn spawn a new thread. This thread will
///   execute a tight loop (see the `Run()` method), making the simulation
///   itself move forward. We will call this thread the RunThread.
///   - When the SimulatorRunner destructor is called, it will try to join the
///   RunThread into the MainThread, to perform a clean shutdown. To do that
///   a boolean flag is set to exit the tight loop in `Run()` and the `join()`
///   method is called on the RunThread, effectively blocking the MainThread
///   until RunThread is done.
///   - When calling a Python-defined callback function from C++ we need to
///   acquire the GIL (Python's Global Interpreter Lock), which basically
///   gives us thread-safety between the C++ and Python worlds. The typical
///   steps to invoke a Python callback from C++ are:
///     - Acquire the GIL.
///     - Invoke the callback.
///     - Release the GIL.
///   Note that acquiring the GIL is a potentially blocking call, as it is
///   basically getting the lock of a mutex.
///
/// Now, the following interleaving may occur when running a python-scripted
/// simulation:
///
/// - Create the simulator runner and configure a python callback.
/// - Start the simulator. By this time we have the MainThread (which is the
///   same thread that executes the python code) and the RunThread executing.
/// - Stop the simulator from the MainThread (calling the `Stop()` method) and
///   consider the following events:
///     - The RunThread yielded the processor right before acquiring the GIL.
///     - The MainTread execute the `Stop()` method and then the destructor.
///       Now the MainThread is waiting for RunThread to join.
///     - Execution is now returned to RunThread, which tries to acquire the
///       GIL. However, the Python thread is blocked on the `join()`, so we
///       are effectively in a deadlock.
///
///   It is interesting to note however that the lock on the GIL happens only
///   if the code is blocked on the C++ side; in other words, if the code was
///   blocked on the Python side (e.g. due to a `sleep` call), C++ has no
///   problem to acquire the GIL. Because of this, the current recommendation
///   is to add a sleep after calling `stop()` from the Python side, so the
///   processor is yielded and the RunThread can finish its current (and last)
///   loop before exiting the while.
class SimulatorRunner {
 public:
  // @brief On agent collision callback function type.
  // @see AutomotiveSimulator::GetCollisions()
  using CollisionCallback =
      std::function<void(const std::vector<AgentCollisionPair<double>>&)>;

  /// @brief Default constructor.
  ///
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  ///
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  ///
  /// @param[in] realtime_rate. Desired rate relative to real time. See
  /// documentation of Simulator::set_target_realtime_rate.
  ///
  /// @param[in] paused A boolean value that if true, will start the
  /// simulator in paused mode.
  /// @param[in] log A boolean value that if true, will log messages
  /// to disk.
  /// @param[in] logfile_name A string with a custom file name for the log.
  SimulatorRunner(std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
                  double time_step, double realtime_rate, bool paused, bool log,
                  std::string logfile_name);

  /// @brief Simplified constructor that starts the simulator at a real-time
  /// rate of 1.0.
  ///
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  ///
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  ///
  /// @param[in] paused A boolean value that if true, will start the
  /// simulator in paused mode.
  /// @param[in] log A boolean value that if true, will log messages
  /// to disk.
  SimulatorRunner(std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
                  double time_step, bool paused, bool log);

  /// @brief Simplified constructor that starts the simulator at a real-time
  /// rate of 1.0.
  ///
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  ///
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  ///
  /// @param[in] paused A boolean value that if true, will start the
  /// simulator in paused mode.
  /// @param[in] log A boolean value that if true, will log messages
  /// to disk.
  /// @param[in] logfile_name A string with a custom file name for the log.
  SimulatorRunner(std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
                  double time_step, bool paused, bool log,
                  std::string logfile_name);

  /// @brief Simplified constructor that starts the simulator with
  /// _paused = false, and log = true.
  ///
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  ///
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  ///
  /// @param[in] realtime_rate. Desired rate relative to real time. See
  /// documentation of Simulator::set_target_realtime_rate.
  SimulatorRunner(std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
                  double time_step, double realtime_rate);

  /// @brief Simplified constructor that starts the simulator with
  /// _paused = false, a real-time rate of 1.0, and log = true.
  ///
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  ///
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  SimulatorRunner(std::unique_ptr<delphyne::AutomotiveSimulator<double>> sim,
                  double time_step);

  /// @brief Default destructor.
  virtual ~SimulatorRunner();

  /// @brief Adds a python callback to be invoked on each simulation step. It is
  /// important to note that the simulation step will be effectively blocked
  /// by this the execution of the callbacks, so please consider this when
  /// adding it.
  ///
  /// @param[in] callable A pointer to a callback function, coming from the
  /// python world.
  void AddStepCallback(std::function<void()> callable);

  /// @brief Adds a callback to be invoked on agent collision.
  ///
  /// @note For collisions to be computed in the first place, collision
  ///       detection must be enabled (via EnableCollisions()).
  /// @note The simulation step will be effectively blocked
  ///       during the sequential execution of all registered
  ///       callbacks.
  /// @param[in] callable The callback function.
  void AddCollisionCallback(CollisionCallback callable);

  /// @brief Spawns a new thread that runs the interactive simulation loop.
  ///
  /// @pre The simulation loop should not be running.
  void Start();

  /// @brief Spawns a new thread that runs the interactive simulation loop for
  /// the provided time period.
  ///
  /// @param[in] duration The duration that the interactive simulation loop
  /// should run for. Note that the time stated here is simulation time,
  /// expressed in seconds.
  ///
  /// @param[in] callback A callback function that will be executed when the
  /// simulation has finished.
  void RunAsyncFor(double duration, std::function<void()> callback);

  /// @brief Runs the interactive simulation loop for the provided time period.
  /// Note that this is a synchronous call and the caller will be blocked until
  /// the interactive simulation loop is done.
  ///
  /// @param[in] duration The duration that the interactive simulation loop
  /// should run for. Note that the time stated here is simulation time,
  /// expressed in seconds.
  void RunSyncFor(double duration);

  /// @brief Stops the thread that is running the interactive simulation loop.
  ///
  /// @pre The interactive simulation loop should be running, either because of
  /// a call to Start() or RunAsyncFor().
  void Stop();

  /// @brief Enqueue a requests for a simulation step to be executed. For this
  /// call to succeed the interactive simulation loop must be running and the
  /// simulation must be paused.
  ///
  /// @pre Start() or RunAsyncFor() has been called.
  /// @pre Paused() has been called or the simulation runner has started paused.
  void RequestSimulationStepExecution(unsigned int steps);

  /// @brief Returns if the interactive simulation loop is currently running or
  /// not.
  bool IsInteractiveLoopRunning() const { return interactive_loop_running_; }

  /// See documentation of Simulator::set_target_realtime_rate()
  void SetRealtimeRate(double realtime_rate);

  /// See documentation of Simulator::get_target_realtime_rate()
  double GetRealtimeRate() const { return realtime_rate_; }

  /// @brief Returns the paused state of the simulation.
  bool IsSimulationPaused() const { return paused_; }

  ///  @brief Pauses the simulation, no-op if called multiple times.
  void PauseSimulation();

  ///  @brief Unauses the simulation, no-op if called multiple times.
  void UnpauseSimulation();

  /// @brief Enables collisions during the simulation, no-op if
  /// called multiple times.
  /// @note Simulation will be paused on collision detection.
  void EnableCollisions() { collisions_enabled_ = true; }

  /// @brief Disables collisions during the simulation, no-op if
  /// called multiple times.
  void DisableCollisions() { collisions_enabled_ = false; }

  /// @brief Returns the current simulation time in seconds.
  double get_current_simulation_time() const {
    return simulator_->get_current_simulation_time();
  }

  const delphyne::AutomotiveSimulator<double>& GetSimulator() const {
    return *simulator_;
  }

  delphyne::AutomotiveSimulator<double>* GetMutableSimulator() {
    return simulator_.get();
  }

  /// @brief Returns the collected interactive simulation statistics
  const InteractiveSimulationStats& get_stats() const { return stats_; }

  /// @brief Returns the logging state. True indicates that logging is enabled.
  bool IsLogging() const { return logging_; }

  /// @brief Start logging to a given file or path.
  void StartLogging(const std::string& filename);

  /// @brief Start logging to a default named file.
  void StartLogging();

  /// @brief Stop logging.
  void StopLogging();

  /// @brief Get the log file name, or empty string if logging has not
  /// been started.
  std::string GetLogFilename() const;

  // @brief The service offered to control the simulation.
  static constexpr char const* kControlService = "/world_control";

  // @brief The topic used to publish notifications.
  static constexpr char const* kNotificationsTopic = "/notifications";

  // @brief The topic used to publish world stats.
  static constexpr char const* kWorldStatsTopic = "/world_stats";

  // @brief The service used when receiving a scene request.
  static constexpr char const* kSceneRequestServiceName = "/get_scene";

 private:
  // @brief Runs the interactive simulation loop for the provided time period.
  // Note that this is a blocking call (i.e. it does not spawn a new thread to
  // run)
  //
  // @param[in] duration The duration that the simulation loop should run for.
  // Note that the time stated here is simulation time, expressed in seconds.
  //
  // @param[in] callback A callback function that will be executed when the
  // simulation has finished.
  void RunInteractiveSimulationLoopFor(double duration,
                                       std::function<void()> callback);

  // @brief Performs a single interactive simulation loop step. This means:
  //
  // - Checking for possible ignition messages coming from the topics that
  // connect the simulation backend to the external world.
  // - Attempting to run a simulation step. This will happen either if the
  // simulation is not paused or there is a queued request to perform a
  // simulation step. Note also that the amount time to advance the simulation
  // is provided by the `time_step_` field and the ratio between real time and
  // simulation time is provided by the configured real-time rate.
  // - Sending the required responses (if any) to outgoing ignition topics
  // that connect the simulation backend to the external world.
  //
  // Note: this method is low-level functionality of the simulator runner and
  // it is the caller responsibility to do the proper invocation to
  // SetupNewRunStats() before calling this method.
  //
  // Finally, note that this is a synchronous call and the caller will be
  // blocked until the interactive simulation step is done.
  void RunInteractiveSimulationLoopStep();

  // @brief Advances the simulation time by the given @p time_step increment
  // in seconds. If necessary, pause the calling thread to be in sync with the
  // configured real-time rate. Update also the simulation statistics.
  //
  // @param[in] time_step The amount of time (in seconds) that the simulation
  // must be advanced.
  void StepSimulationBy(double time_step);

  // @brief Process one SceneRequest message.
  //
  // @param[in] msg The message
  void ProcessSceneRequest(const ignition::msgs::SceneRequest& msg);

  // @brief Service used to receive scene request messages.
  //
  // @param[in] request The request.
  //
  // @param[out] response The response (unused).
  // @return The result of the service.
  bool OnSceneRequest(
      const ignition::msgs::SceneRequest& request,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      ignition::msgs::Boolean& response);

  // @brief Processes one WorldControl message.
  //
  // @param[in] msg The message
  void ProcessWorldControlMessage(const ignition::msgs::WorldControl& msg);

  // @brief Service used to receive world control messages.
  //
  // @param[in] request The request.
  //
  // @param[out] response The response (unused).
  // @return The result of the service.
  bool OnWorldControl(
      const ignition::msgs::WorldControl& request,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      ignition::msgs::Boolean& response);

  // @brief Processes all pending incoming messages.
  void ProcessIncomingMessages();

  // @brief Sends all outgoing messages.
  void SendOutgoingMessages();

  // @brief Sends all world stats (whether the world is paused for now).
  void SendWorldStats();

  // @brief Stores the old stats and prepares a clean one for a new run.
  void SetupNewRunStats();

  // @brief The time (seconds) that we simulate in each simulation step.
  const double time_step_;

  // @brief Whether the main loop has been started or not.
  std::atomic<bool> interactive_loop_running_{false};

  // @brief A pointer to the Drake simulator.
  std::unique_ptr<delphyne::AutomotiveSimulator<double>> simulator_;

  // @brief Whether an external step was requested or not.
  unsigned int steps_requested_{0u};

  // @brief A mutex to avoid races.
  std::mutex mutex_;

  // @brief The thread in charge of doing all the periodic tasks.
  std::thread main_thread_;

  // @brief A queue for storing the incoming messages (requests).
  std::queue<ignition::msgs::SimulationInMessage> incoming_msgs_;

  // @brief A queue for storing the outgoing messages (notifications).
  std::queue<ignition::msgs::WorldControl> outgoing_msgs_;

  // @brief An Ignition Transport node used for communication.
  ignition::transport::Node node_;

  // @brief An Ignition Transport publisher for sending notifications.
  ignition::transport::Node::Publisher notifications_pub_;

  // @brief An Ignition Transport publisher for sending world stats.
  ignition::transport::Node::Publisher world_stats_pub_;

  // @brief A vector that holds all the registered callbacks that need to be
  // triggered on each simulation step.
  std::vector<std::function<void()>> step_callbacks_;

  // @brief A vector that holds all the registered callbacks that need to be
  // triggered on agent collision.
  std::vector<CollisionCallback> collision_callbacks_;

  // @brief Whether collisions are checked for during simulation or not.
  bool collisions_enabled_{false};

  // @brief The period between world statistics updates (ms).
  const double kWorldStatsPeriodMs_ = 250.0;

  // @brief The last time that the scene message was updated.
  std::chrono::steady_clock::time_point last_world_stats_update_;

  // @brief Slow down the simulation to this rate if possible (user settable).
  double realtime_rate_{1.0};

  // @brief Whether the simulation is paused or not.
  bool paused_{false};

  // @brief The statistics of the (possibly many) simulation runs.
  InteractiveSimulationStats stats_;

  // @brief An Ignition Transport log recorder object.
  ignition::transport::log::Recorder recorder_;

  // @brief Whether the simulation is logging or not.
  bool logging_{false};
};

}  // namespace delphyne
