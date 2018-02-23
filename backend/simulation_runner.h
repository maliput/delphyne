
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

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "backend/automotive_simulator.h"
#include "backend/system.h"

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <protobuf/robot_model_request.pb.h>
#include <protobuf/simulation_in_message.pb.h>

#include <pybind11/pybind11.h>

#include <Python.h>

namespace delphyne {
namespace backend {
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
  /// @brief Default constructor.
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  /// @param[in] realtime_rate. Desired rate relative to real time. See
  /// documentation of Simulator::set_target_realtime_rate.
  /// @param[in] paused A boolean value that if true, will start the
  /// simulator in paused mode.
  SimulatorRunner(
      std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
      double time_step, double realtime_rate, bool paused);

  /// @brief Simplified constructor that starts the simulator at a real-time
  /// rate of 1.0.
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  /// @param[in] paused A boolean value that if true, will start the
  /// simulator in paused mode.
  SimulatorRunner(
      std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
      double time_step, bool paused);

  /// @brief Simplified constructor that starts the simulator with
  /// _paused = false.
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  /// @param[in] realtime_rate. Desired rate relative to real time. See
  /// documentation of Simulator::set_target_realtime_rate.

  SimulatorRunner(
      std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
      double time_step, double realtime_rate);

  /// @brief Simplified constructor that starts the simulator with
  /// _paused = false and a real-time rate of 1.0.
  /// @param[in] sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  /// @param[in] time_step The slot of time (seconds) simulated in each
  /// simulation step.
  SimulatorRunner(
      std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> sim,
      double time_step);

  /// @brief Default destructor.
  virtual ~SimulatorRunner();

  /// @brief Adds a python callback to be invoked on each simulation step. It is
  /// important to note that the simulation step will be effectively blocked
  /// by this the execution of the callbacks, so please consider this when
  /// adding it.
  /// @param[in] callable A pointer to a callback function, coming from the
  /// python world.
  void AddStepCallback(std::function<void()> callable);

  /// @brief Starts the thread that runs the simulation loop. If there was a
  /// previous call to Start(), this call will be ignored.
  void Start();

  /// @brief Stops the thread that runs the simulation loop. If there was a
  /// previous call to Stop(), this call will be ignored.
  void Stop();

  /// @brief Runs the main simulation loop.
  void Run();

  /// @brief Advances a single simulation step by time_step_ seconds.
  void RunSimulationStep();

  /// See documentation of AutomotiveSimulator::SetRealtimeRate.
  void SetRealtimeRate(double realtime_rate) {
    simulator_->SetRealtimeRate(realtime_rate);
  }

  /// See documentation of AutomotiveSimulator::GetRealtimeRate.
  double GetRealtimeRate() const { return simulator_->GetRealtimeRate(); }

  /// @brief Returns the paused state of the simulation.
  bool IsPaused() const;

  /// @brief Requests the simulation to execute a single simulation step.
  /// The simulation must be paused before calling this method.
  /// @pre Start() has been called.
  /// @pre Paused() has been called.
  void RequestStep(double time_step);

  ///  @brief Pauses the simulation, no-op if called multiple times.
  void Pause();

  ///  @brief Unauses the simulation, no-op if called multiple times.
  void Unpause();

 private:
  // @brief Process one RobotModelRequest message.
  // @param[in] msg The message
  void ProcessRobotModelRequest(const ignition::msgs::RobotModelRequest& msg);

  // @brief Service used to receive robot model request messages.
  // @param[in] request The request.
  // @param[out] response The response (unused).
  // @return The result of the service.
  bool OnRobotModelRequest(
      const ignition::msgs::RobotModelRequest& request,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      ignition::msgs::Boolean& response);

  // @brief Processes one WorldControl message.
  // @param[in] msg The message
  void ProcessWorldControlMessage(const ignition::msgs::WorldControl& msg);

  // @brief Service used to receive world control messages.
  // @param[in] request The request.
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

  // @brief The service offered to control the simulation.
  const std::string kControlService{"/world_control"};

  // @brief The topic used to publish notifications.
  const std::string kNotificationsTopic{"/notifications"};

  // @brief The topic used to publish world stats.
  const std::string kWorldStatsTopic{"/world_stats"};

  // @brief The service used when receiving a robot request.
  const std::string kRobotRequestServiceName{"/get_robot_model"};

  // @brief The time (seconds) that we simulate in each simulation step.
  const double time_step_;

  // @brief The time (seconds) that we simulate in a custom step requested
  // externally.
  double custom_time_step_{0.001};

  // @brief Whether the main loop has been started or not.
  std::atomic<bool> enabled_{false};

  // @brief A pointer to the Drake simulator.
  std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> simulator_;

  // @brief Whether the simulation is paused or not.
  bool paused_{false};

  // @brief Whether an external step was requested or not.
  bool step_requested_{false};

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

  // The period between world statistics updates (ms).
  const double kWorldStatsPeriodMs_ = 250.0;

  // The last time that the scene message was updated.
  std::chrono::steady_clock::time_point last_world_stats_update_;
};

}  // namespace backend
}  // namespace delphyne
