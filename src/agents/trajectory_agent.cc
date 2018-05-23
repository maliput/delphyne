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

#include <drake/automotive/agent_trajectory.h>
#include <drake/automotive/prius_vis.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "backend/ign_publisher_system.h"
#include "backend/translation_systems/drake_simple_car_state_to_ign.h"
#include "systems/trajectory_follower.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

TrajectoryAgent::TrajectoryAgent(const std::string& name)
    : delphyne::Agent(name), trajectory_follower_system_() {
  std::cout << "Trajectory Agent Constructor" << std::endl;
  igndbg << "TrajectoryAgent constructor" << std::endl;
}

int TrajectoryAgent::Configure(
    const int& id, drake::systems::DiagramBuilder<double>* builder,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "TrajectoryAgent configure" << std::endl;

  /*********************
   * Basics
   *********************/
  id_ = id;

  /*********************
   * Parameters
   *********************/
  //   std::vector<double> times2 =
  //       linb::any_cast<std::vector<double>>(
  //           parameters.at("times"));
  //   std::vector<double> headings =
  //       linb::any_cast<std::vector<double>>(
  //           parameters.at("headings"));
  //   std::vector<std::vector<double>> translations_from_parameters =
  //       linb::any_cast<std::vector<std::vector<double>>>(
  //           parameters.at("translations"));
  // ABORT ABORT ABORT!
  // The above works, but it will be much cleaner to provide python bindings
  // to helper functions outside this that create a trajectory and then
  // pass in AgentTrajectory as one of the parameters. Otherwise trajectory
  // making will be constrained to whatever this class is capable of
  /*********************
   * Trajectory
   *********************/
  std::vector<double> times{0.0, 5.0, 10.0, 15.0, 20.0};
  Eigen::Quaternion<double> zero_heading(
      Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitZ()));
  std::vector<Eigen::Quaternion<double>> orientations(5, zero_heading);
  double y = -5.55;
  std::vector<Eigen::Vector3d> translations{
      Eigen::Vector3d(0.0, y, 0.0), Eigen::Vector3d(10.0, y, 0.0),
      Eigen::Vector3d(30.0, y, 0.0), Eigen::Vector3d(60.0, y, 0.0),
      Eigen::Vector3d(100.0, y, 0.0)};
  drake::automotive::AgentTrajectory trajectory =
      drake::automotive::AgentTrajectory::Make(times, orientations,
                                               translations);

  /*********************
   * Instantiate System
   *********************/
  double sampling_time = 0.01;
  std::unique_ptr<drake::automotive::TrajectoryFollower<double>> system =
      std::make_unique<drake::automotive::TrajectoryFollower<double>>(
          trajectory, sampling_time);
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
  auto ports = aggregator->AddSinglePoseAndVelocityInput(name_, id);
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
  std::cout << "Finished" << std::endl;
  return 0;
}

int TrajectoryAgent::Initialize(drake::systems::Context<double>* context) {
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
