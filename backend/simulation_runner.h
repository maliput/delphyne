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

#ifndef DELPHYNE_BRIDGE_SIMULATIONRUNNER_HH_
#define DELPHYNE_BRIDGE_SIMULATIONRUNNER_HH_

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <drake/automotive/automotive_simulator.h>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "protobuf/simulation_in_message.pb.h"

namespace delphyne {
namespace backend {

/// \brief Block the current thread until a SIGINT or SIGTERM is received.
/// Note that this function registers a signal handler. Do not use this
/// function if you want to manage yourself SIGINT/SIGTERM.
void WaitForShutdown();

/// \brief A wrapper to execute the Drake simulator as a back-end.
/// This class exposes some of the functionality of the simulator via Ignition
/// Transport services (e.g.: play/pause/step the simulation). At the same time,
/// it has to periodically advance the simulation a certain amount of time.
/// Finally, it notifies any given state changes to the outside world via
/// Ignition Transport messages. All these operations occur in a main loop with
/// the following structure:
///
/// * Process incoming messages: All incoming requestes are queued
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
class SimulatorRunner {
  /// \brief Default constructor.
  /// \param[in] _sim A pointer to a simulator. Note that we take ownership of
  /// the simulation.
  /// \param[in] _timeStep The slot of time (seconds) simulated in each
  /// simulation step.
 public:
  SimulatorRunner(
      std::unique_ptr<drake::automotive::AutomotiveSimulator<double>> _sim,
      const double _timeStep);

  /// \brief Default destructor.
 public:
  virtual ~SimulatorRunner();

  /// \brief Start the thread that runs the simulation loop. If there was a
  /// previous call to Start(), this call will be ignored.
 public:
  void Start();

  /// \brief Run the main simulation loop.
 public:
  void Run();

  /// \brief Process all pending incoming messages.
 private:
  void ProcessIncomingMessages();

  /// \brief Send all outgoing messages.
 private:
  void SendOutgoingMessages();

  /// \brief Process one WorldControl message.
  /// \param[in] _msg The message
 private:
  void ProcessWorldControlMessage(const ignition::msgs::WorldControl& _msg);

  /// \brief Service used to receive simulation input messages.
  /// \param[in] _req The request.
  /// \param[out] _rep The response (unused).
  /// \param[out] _result The result of the service.
 private:
  void OnSimulationInMessage(
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      const ignition::msgs::SimulationInMessage& _req,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      ignition::msgs::Boolean& _rep, bool& _result);

  /// \brief Get the default time step.
  /// \return The default time step.
 private:
  double TimeStep() const;

  /// \brief Set the default time step.
  /// \param[in] _timeStep The new time step.
 private:
  void SetTimeStep(const double _timeStep);

  /// \brief Get whether the simulation is paused or not.
  /// \return True when the simulation is paused or false otherwise.
 private:
  bool IsPaused() const;

  /// \brief Pause/unpause the simulation.
  /// \param[in] _paused True for paused, false for unpaused.
 private:
  void SetPaused(const bool _paused);

  /// \brief Whether an external step was requested or not.
  /// \return True if requested or false otherwise.
 private:
  bool StepRequested() const;

  /// \brief Set whether an external step is requested or not.
  /// \param[in] _stepRequested True when a step is requested.
 private:
  void SetStepRequested(const bool _stepRequested);

  /// \brief Get the custom time step for an external step.
 private:
  double CustomTimeStep() const;

  /// \brief Set the custom time step for an external step.
  /// \param[in] _timeStep The custom time step.
 private:
  void SetCustomTimeStep(const double _timeStep);

  /// \brief The service offered to control the simulation.
 private:
  const std::string kControlService = "/world_control";

  /// \brief The topic used to publish notifications.
 private:
  const std::string kNotificationsTopic = "/notifications";

  /// \brief The time (seconds) that we simulate in each simulation step.
 private:
  double timeStep = 0.001;

  /// \brief The time (seconds) that we simulate in a custom step requested
  /// externally.
 private:
  double customTimeStep = 0.001;

  /// \brief Whether the main loop has been started or not.
 private:
  bool enabled = false;

  /// \brief A pointer to the Drake simulator.
 private:
  std::unique_ptr<drake::automotive::AutomotiveSimulator<double>> simulator;

  /// \brief Whether the simulation is paused or not.
 private:
  bool paused = false;

  /// \brief Whether an external step was requested or not.
 private:
  bool stepRequested = false;

  /// \brief A mutex to avoid races.
 private:
  std::mutex mutex;

  /// \brief The thread in charge of doing all the periodic tasks.
 private:
  std::thread mainThread;

  /// \brief A queue for storing the incoming messages (requests).
 private:
  std::queue<ignition::msgs::SimulationInMessage> incomingMsgs;

  /// \brief A queue for storing the outgoing messages (notifications).
 private:
  std::queue<ignition::msgs::WorldControl> outgoingMsgs;

  /// \brief An Ignition Transport node used for communication.
 private:
  ignition::transport::Node node;

  /// \brief An Ignition Transport publisher for sending notifications.
 private:
  ignition::transport::Node::Publisher notificationsPub;
};

}  // namespace backend
}  // namespace delphyne

#endif
