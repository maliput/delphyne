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

#include <maliput/api/road_network.h>

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
class TrajectoryAgentBlueprint : public BasicAgentBlueprint {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryAgentBlueprint)

  TrajectoryAgentBlueprint(const std::string& name, const std::vector<double>& times,
                           const std::vector<double>& headings, const std::vector<std::vector<double>>& translations);

 private:
  std::unique_ptr<Agent::Diagram> DoBuildDiagram(maliput::api::RoadNetwork* road_network) const override;

  std::unique_ptr<Trajectory> trajectory_{};
};

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
