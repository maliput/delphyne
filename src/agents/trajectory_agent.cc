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

#include <drake/automotive/prius_vis.h>
#include <drake/automotive/trajectory.h>
#include <drake/automotive/trajectory_follower.h>
#include <drake/common/eigen_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// public headers
#include "delphyne/macros.h"

// private headers
#include "agents/helpers/geometry_wiring.h"
#include "backend/ign_publisher_system.h"
#include "translations/drake_simple_car_state_to_ign.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

// TODO(daniel.stonier) convert this to accepting a Trajectory class instead
TrajectoryAgent::TrajectoryAgent(
    const std::string& name, const std::vector<double>& times,
    const std::vector<double>& headings,
    const std::vector<std::vector<double>>& translations)
    : delphyne::Agent(name), initial_time_(times.back()) {
  igndbg << "TrajectoryAgent constructor" << std::endl;
  Eigen::Quaternion<double> zero_heading(
      Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitZ()));

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

  trajectory_ = std::make_unique<drake::automotive::Trajectory>(
      drake::automotive::Trajectory::Make(times, eigen_orientations,
                                          eigen_translations));
}

void TrajectoryAgent::Configure(
    int id, const drake::maliput::api::RoadGeometry* road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "TrajectoryAgent configure" << std::endl;

  /*********************
   * Checks
   *********************/
  DELPHYNE_VALIDATE(builder != nullptr, std::invalid_argument,
                    "Builder must not be null");
  DELPHYNE_VALIDATE(scene_graph != nullptr, std::invalid_argument,
                    "Scene graph must not be null");
  DELPHYNE_VALIDATE(aggregator != nullptr, std::invalid_argument,
                    "Aggregator must not be null");
  DELPHYNE_VALIDATE(car_vis_applicator != nullptr, std::invalid_argument,
                    "Car visualization applicator must not be null");

  /*********************
   * Basics
   *********************/
  id_ = id;

  /******************************************
   * Trajectory Follower System
   ******************************************/
  // TODO(daniel.stonier) have this sample on update events from the simulation
  // than arbitrarily choosing it's own update rate.
  double sampling_time = 0.01;

  typedef drake::automotive::TrajectoryFollower<double> TrajectoryFollower;
  TrajectoryFollower* trajectory_follower_system =
      builder->template AddSystem<TrajectoryFollower>(
          std::make_unique<TrajectoryFollower>(*trajectory_, sampling_time));
  trajectory_follower_system->set_name(name_);

  /*********************
   * Simulator Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  drake::systems::rendering::PoseVelocityInputPortDescriptors<double> ports =
      aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(trajectory_follower_system->pose_output(),
                   ports.pose_descriptor);
  builder->Connect(trajectory_follower_system->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id, name_));

  // Computes the initial world to car transform X_WC0.
  const drake::automotive::PoseVelocity initial_car_pose_velocity =
      trajectory_->value(initial_time_);
  const drake::Isometry3<double> X_WC0 =
      drake::Translation3<double>(initial_car_pose_velocity.translation())
      * initial_car_pose_velocity.rotation();

  // Wires up the Prius geometry.
  builder->Connect(trajectory_follower_system->pose_output(), WirePriusGeometry(
      name_, X_WC0, builder, scene_graph, &geometry_ids_));

  /*********************
   * State Publisher
   *********************/
  auto agent_state_translator =
      builder->template AddSystem<DrakeSimpleCarStateToIgn>();

  const std::string agent_state_channel =
      "agents/" + std::to_string(id) + "/state";
  typedef IgnPublisherSystem<ignition::msgs::SimpleCarState>
      AgentStatePublisherSystem;
  AgentStatePublisherSystem* agent_state_publisher_system =
      builder->template AddSystem<AgentStatePublisherSystem>(
          std::make_unique<AgentStatePublisherSystem>(agent_state_channel));

  // Drake car states are translated to ignition.
  builder->Connect(trajectory_follower_system->state_output(),
                   agent_state_translator->get_input_port(0));

  // And then the translated ignition car state is published.
  builder->Connect(*agent_state_translator, *agent_state_publisher_system);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
