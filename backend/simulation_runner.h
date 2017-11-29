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

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "backend/automotive_simulator.h"
#include "protobuf/robot_model_request.pb.h"
#include "protobuf/simulation_in_message.pb.h"

namespace delphyne {
namespace backend {

// \brief Block the current thread until a SIGINT or SIGTERM is received.
// Note that this function registers a signal handler. Do not use this
// function if you want to manage yourself SIGINT/SIGTERM.
void WaitForShutdown();

// \brief A wrapper to execute the Drake simulator as a back-end.
// This class exposes some of the functionality of the simulator via Ignition
// Transport services (e.g.: play/pause/step the simulation). At the same time,
// it has to periodically advance the simulation a certain amount of time.
// Finally, it notifies any given state changes to the outside world via
// Ignition Transport messages. All these operations occur in a main loop with
// the following structure:
//
// * Process incoming messages: All incoming requestes are queued
//   asynchronously when received and in this step we process all of them.
//   This request might involve flag the simulation as paused, insert a new
//   model, remove an existing model, etc.
//
// * Step the simulation: We advance the simulation a certain amount of time.
//   The amount of time could be the default time step (if we are in play
//   mode), a custom time step (if we are externally stepping) or no time at
//   all if the simulation is paused.
//
// * Notify changes in the simulation. The simulation will send messages over a
//   a well-known topic every time that a relevant simulation state changed
//   occur. E.g.: When the simulation is paused or a model is removed. These
//   notifications should be used by all the Visualizer instances to update its
//   state/widgets in order to keep them in sync with the back-end.
class SimulatorRunner {
  // \brief Default constructor.
  // \param[in] _sim A pointer to a simulator. Note that we take ownership of
  // the simulation.
  // \param[in] _timeStep The slot of time (seconds) simulated in each
  // simulation step.
 public:
  SimulatorRunner(
      std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> _sim,
      double _timeStep);

  // \brief Default destructor.
  virtual ~SimulatorRunner();

  // \brief Start the thread that runs the simulation loop. If there was a
  // previous call to Start(), this call will be ignored.
  void Start();

  // \brief Run the main simulation loop.
  void Run();

 private:
  // \brief Process all pending incoming messages.
  void ProcessIncomingMessages();

  // \brief Send all outgoing messages.
  void SendOutgoingMessages();

  // \brief Process one WorldControl message.
  // \param[in] _msg The message
  void ProcessWorldControlMessage(const ignition::msgs::WorldControl& _msg);

  // \brief Process one RobotModelRequest message.
  // \param[in] _msg The message
  void ProcessRobotModelRequest(const ignition::msgs::RobotModelRequest& _msg);

  /// \brief Service used to receive simulation input messages.
  /// \param[in] _req The request.
  /// \param[out] _rep The response (unused).
  /// \param[out] _result The result of the service.
 private:
  void OnRobotModelRequest(
      const ignition::msgs::RobotModelRequest& request,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      ignition::msgs::Boolean& response,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      bool& result);

  void OnWorldControl(
      const ignition::msgs::WorldControl& request,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      ignition::msgs::Boolean& response,
      // NOLINTNEXTLINE(runtime/references) due to ign-transport API
      bool& result);

  // \brief Get the default time step.
  // \return The default time step.
  double TimeStep() const;

  // \brief Set the default time step.
  // \param[in] _timeStep The new time step.
  void SetTimeStep(const double _timeStep);

  // \brief Get whether the simulation is paused or not.
  // \return True when the simulation is paused or false otherwise.
  bool IsPaused() const;

  // \brief Pause/unpause the simulation.
  // \param[in] _paused True for paused, false for unpaused.
  void SetPaused(const bool _paused);

  // \brief Whether an external step was requested or not.
  // \return True if requested or false otherwise.
  bool StepRequested() const;

  // \brief Set whether an external step is requested or not.
  // \param[in] _stepRequested True when a step is requested.
  void SetStepRequested(const bool _stepRequested);

  // \brief Get the custom time step for an external step.
  double CustomTimeStep() const;

  // \brief Set the custom time step for an external step.
  // \param[in] _timeStep The custom time step.
  void SetCustomTimeStep(const double _timeStep);

  // \brief The service offered to control the simulation.
  const std::string kControlService = "/world_control";

  // \brief The topic used to publish notifications.
  const std::string kNotificationsTopic = "/notifications";

  // \brief The time (seconds) that we simulate in each simulation step.
  double timeStep = 0.001;

  // \brief The time (seconds) that we simulate in a custom step requested
  // externally.
  double customTimeStep = 0.001;

  // \brief Whether the main loop has been started or not.
  bool enabled = false;

  // \brief A pointer to the Drake simulator.
  std::unique_ptr<delphyne::backend::AutomotiveSimulator<double>> simulator;

  // \brief Whether the simulation is paused or not.
  bool paused = false;

  // \brief Whether an external step was requested or not.
  bool stepRequested = false;

  // \brief A mutex to avoid races.
  std::mutex mutex;

  // \brief The thread in charge of doing all the periodic tasks.
  std::thread mainThread;

 private:
  // \brief A queue for storing the incoming messages (requests).
  std::queue<ignition::msgs::SimulationInMessage> incomingMsgs;

  // \brief A queue for storing the outgoing messages (notifications).
  std::queue<ignition::msgs::WorldControl> outgoingMsgs;

  // \brief An Ignition Transport node used for communication.
  ignition::transport::Node node;

  // \brief An Ignition Transport publisher for sending notifications.
  ignition::transport::Node::Publisher notificationsPub;
};

}  // namespace backend
}  // namespace delphyne
