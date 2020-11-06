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

#include <maliput/api/road_geometry.h>
#include "delphyne/mi6/agent_base_blueprint.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// @brief A very simple vehicle agent that can be teleoperated.
class UnicycleCarBlueprint : public BasicAgentBlueprint {
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

  std::unique_ptr<Agent::Diagram> DoBuildDiagram(const maliput::api::RoadGeometry* road_geometry) const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
