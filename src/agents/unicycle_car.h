/**
 * @file src/agents/simple_car.h
 *
 * Copyright 2020 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <maliput/api/road_network.h>

#include "delphyne/mi6/agent_base_blueprint.h"
#include "systems/vector_source.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// @brief An agent that follows roads as if they were railroad tracks.
///
/// The underlying road network has a reference line for each lane which
/// is utilised by this agent as a railroad track.
class UnicycleCarAgent : public Agent {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(UnicycleCarAgent)

  /// Unicycle car agent constructor.
  ///
  /// @param diagram The Diagram representation of the rail car agent.
  /// @param acceleration_setter The acceleration input to the underlying UnicycleCar
  /// in this diagram.
  /// @param angular_rate_setter The angular rate input to the underlying UnicycleCar
  /// in this diagram.

  explicit UnicycleCarAgent(Agent::Diagram* diagram, VectorSource<double>* acceleration_setter,
                            VectorSource<double>* angular_rate_setter)
      : Agent(diagram), acceleration_setter_(acceleration_setter), angular_rate_setter_(angular_rate_setter) {}

  /// Sets the acceleration input to this agent.
  ///
  /// @param new_acceleration The new acceleration input to the agent.
  void SetAcceleration(double new_acceleration) { acceleration_setter_->Set(new_acceleration); }

  /// Sets the angular rate input to this agent.
  ///
  /// @param new_angular_rate The new angular rate input to the agent.
  void SetAngularRate(double new_angular_rate) { angular_rate_setter_->Set(new_angular_rate); }

 private:
  VectorSource<double>* acceleration_setter_;
  VectorSource<double>* angular_rate_setter_;
};

/// @brief A very simple vehicle agent with settable inputs.
class UnicycleCarBlueprint : public TypedAgentBlueprint<UnicycleCarAgent> {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(UnicycleCarBlueprint)

  explicit UnicycleCarBlueprint(const std::string& name, double x, double y, double heading, double speed);

 private:
  // Container for the agent's initial configuration.
  //
  // Note: this is independent of whatever computational mechanisms
  // are used internally and is a useful construct for recording and
  // logging / streaming to debug configuration errors.
  struct InitialConditions {
    double x{0.0};        // scene x-coordinate (m)
    double y{0.0};        // scene y-coordinate (m)
    double heading{0.0};  // scene heading (around z-axis (radians)
    double speed{0.0};    // speed in axis defined by the heading (m/s)
    InitialConditions(double x, double y, double heading, double speed) : x(x), y(y), heading(heading), speed(speed) {}
  } initial_conditions_;

  std::unique_ptr<UnicycleCarAgent> DoBuildAgentInto(
      const maliput::api::RoadNetwork* road_network,
      drake::systems::DiagramBuilder<double>* simulator_builder) const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
