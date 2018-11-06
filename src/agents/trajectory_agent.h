/**
 * @file src/agents/trajectory_agent.h
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <drake/automotive/maliput/api/road_geometry.h>
#include "delphyne/mi6/agent_base_blueprint.h"
#include "systems/trajectory.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Interfaces
 *****************************************************************************/

/// @brief Trajectory following agents
class TrajectoryAgentBlueprint : public SimpleAgentBlueprint {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryAgentBlueprint)

  TrajectoryAgentBlueprint(
      const std::string& name,
      const std::vector<double>& times,
      const std::vector<double>& headings,
      const std::vector<std::vector<double>>& translations);

 private:
  std::unique_ptr<Agent::Diagram> DoBuildDiagram(
      const drake::maliput::api::RoadGeometry* road_geometry) const override;

  std::unique_ptr<Trajectory> trajectory_{};
};

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
