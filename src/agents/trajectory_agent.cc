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

#include <Eigen/Core>
#include <Eigen/Geometry>

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
    : delphyne::Agent(name), trajectory_(), trajectory_follower_system_() {
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

int TrajectoryAgent::Configure(
    int id, const drake::maliput::api::RoadGeometry* road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "TrajectoryAgent configure" << std::endl;

  /*********************
   * Basics
   *********************/
  id_ = id;

  /*********************
   * Instantiate System
   *********************/
  // TODO(daniel.stonier) have this sample on update events from the simulation
  // than arbitrarily choosing it's own update rate.
  double sampling_time = 0.01;
  std::unique_ptr<drake::automotive::TrajectoryFollower<double>> system =
      std::make_unique<drake::automotive::TrajectoryFollower<double>>(
          *trajectory_, sampling_time);
  system->set_name(name_);
  trajectory_follower_system_ =
      builder
          ->template AddSystem<drake::automotive::TrajectoryFollower<double>>(
              std::move(system));

  /*********************
   * Diagram Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  drake::systems::rendering::PoseVelocityInputPortDescriptors<double> ports =
      aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(trajectory_follower_system_->pose_output(),
                   ports.pose_descriptor);
  builder->Connect(trajectory_follower_system_->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id, name_));

  /*********************
   * State Publisher
   *********************/
  auto agent_state_translator =
      builder->template AddSystem<DrakeSimpleCarStateToIgn>();

  const std::string agent_state_channel =
      "agents/" + std::to_string(id) + "/state";
  auto agent_state_publisher =
      builder->AddSystem<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
          agent_state_channel);

  // Drake car states are translated to ignition.
  builder->Connect(trajectory_follower_system_->state_output(),
                   agent_state_translator->get_input_port(0));

  // And then the translated ignition car state is published.
  builder->Connect(*agent_state_translator, *agent_state_publisher);
  return 0;
}

int TrajectoryAgent::Initialize(drake::systems::Context<double>* context) {
  // TODO(daniel.stonier) unwind this and pre-declare instead
  igndbg << "TrajectoryAgent initialize" << std::endl;
  return 0;
}

drake::systems::System<double>* TrajectoryAgent::get_system() const {
  return trajectory_follower_system_;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
