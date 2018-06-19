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

#include <drake/automotive/car_vis_applicator.h>
#include <drake/automotive/trajectory.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "delphyne/agent_base.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Interfaces
 *****************************************************************************/

/// @brief Trajectory following agents
class TrajectoryAgent : public delphyne::Agent {
 public:
  TrajectoryAgent(const std::string& name, const std::vector<double>& times,
                  const std::vector<double>& headings,
                  const std::vector<std::vector<double>>& translations);

  int Configure(
      int id, const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<double>* builder,
      drake::geometry::SceneGraph<double>* scene_graph,
      drake::systems::rendering::PoseAggregator<double>* aggregator,
      drake::automotive::CarVisApplicator<double>* car_vis_applicator) override;

 private:
  const double initial_time_{};
  std::unique_ptr<drake::automotive::Trajectory> trajectory_{};
};

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
