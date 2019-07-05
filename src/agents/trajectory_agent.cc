/**
 * @file src/agents/trajectory_agent.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/trajectory_agent.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <drake/common/eigen_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "systems/trajectory.h"
#include "systems/trajectory_follower.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

// TODO(daniel.stonier) convert this to accepting a Trajectory class instead
TrajectoryAgentBlueprint::TrajectoryAgentBlueprint(
    const std::string& name, const std::vector<double>& times,
    const std::vector<double>& headings,
    const std::vector<std::vector<double>>& translations)
    : BasicAgentBlueprint(name) {
  std::vector<Eigen::Quaternion<double>> eigen_orientations;
  for (const double& heading : headings) {
    Eigen::Quaternion<double> orientation(
        Eigen::AngleAxis<double>(heading, Eigen::Vector3d::UnitZ()));
    eigen_orientations.push_back(orientation);
  }
  std::vector<Eigen::Vector3d> eigen_translations;
  for (const std::vector<double>& translation : translations) {
    // TODO(daniel.stonier) assert on size 3, but we'll instead be switching to
    // accepting trajectories here, do it later
    Eigen::Vector3d eigen_translation;
    eigen_translation << translation[0], translation[1], translation[2];
    eigen_translations.push_back(eigen_translation);
  }

  trajectory_ = std::make_unique<Trajectory>(
      Trajectory::Make(times, eigen_orientations, eigen_translations));

  // TODO(daniel.stonier) stop using this, make use of an initial value on
  // the pose output
  const PoseVelocity initial_car_pose_velocity =
      trajectory_->value(times.front());
}

std::unique_ptr<Agent::Diagram> TrajectoryAgentBlueprint::DoBuildDiagram(
    const maliput::api::RoadGeometry* road_geometry) const {
  drake::unused(road_geometry);
  AgentBlueprint::DiagramBuilder builder(this->name());

  /******************************************
   * Trajectory Follower System
   ******************************************/
  // TODO(daniel.stonier) have this sample on update events from the simulation
  // than arbitrarily choosing it's own update rate.
  double sampling_time = 0.01;

  typedef TrajectoryFollower<double> TrajectoryFollower;
  TrajectoryFollower* trajectory_follower_system = builder.AddSystem(
      std::make_unique<TrajectoryFollower>(*trajectory_, sampling_time));
  trajectory_follower_system->set_name(this->name() + "_system");

  /*********************
   * Diagram Outputs
   *********************/
  builder.ExportStateOutput(trajectory_follower_system->state_output());
  builder.ExportPoseOutput(trajectory_follower_system->pose_output());
  builder.ExportVelocityOutput(trajectory_follower_system->velocity_output());

  return builder.Build();
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
