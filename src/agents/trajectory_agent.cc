// Copyright 2017 Toyota Research Institute

//#include <cmath>
//#include <iostream>
//#include <map>
//#include <memory>
//#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include "drake/automotive/agent_trajectory.h"
#include "drake/automotive/prius_vis.h"

#include <backend/ign_publisher_system.h>
#include <backend/translation_systems/drake_simple_car_state_to_ign.h>
#include <trajectory_follower.h>

#include "../../include/delphyne/agent_plugin_base.h"
#include "../../include/delphyne/linb-any"

namespace delphyne {

/**
 * @brief Trajectory following agents
 *
 * @TODO(daniel.stonier) add agent type (for visualisation purpose only)
 */
class TrajectoryAgent : public delphyne::AgentPlugin {

public:
 DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryAgent)

 TrajectoryAgent() : trajectory_follower_system_() {
   igndbg << "TrajectoryAgent constructor" << std::endl;
 }

 int Configure(const std::string& name, const int& id,
               const std::map<std::string, linb::any>& parameters,
               drake::systems::DiagramBuilder<double>* builder,
               drake::systems::rendering::PoseAggregator<double>* aggregator,
               drake::automotive::CarVisApplicator<double>* car_vis_applicator)
     override {
   igndbg << "TrajectoryAgent configure" << std::endl;

   /*********************
    * Basics
    *********************/
   id_ = id;
   name_ = name;

   /*********************
    * Instantiate System
    *********************/
   std::vector<double> times{0.0, 5.0, 10.0, 15.0, 20.0};
   Eigen::Quaternion<double> zero_heading(
       Eigen::AngleAxis<double>(0.0,
                                Eigen::Vector3d::UnitZ()));
   std::vector<Eigen::Quaternion<double>> orientations(5, zero_heading);
   std::vector<Eigen::Vector3d> translations{
     Eigen::Vector3d( 0.0, 0.00, 0.00),
     Eigen::Vector3d( 10.0, 0.00, 0.00),
     Eigen::Vector3d( 30.0, 0.00, 0.00),
     Eigen::Vector3d( 60.0, 0.00, 0.00),
     Eigen::Vector3d(100.0, 0.00, 0.00)
   };
   drake::automotive::AgentTrajectory trajectory =
       drake::automotive::AgentTrajectory::Make(
           times,
           orientations,
           translations
           );
   double sampling_time = 0.01;
   std::unique_ptr<drake::automotive::TrajectoryFollower<double>> system =
       std::make_unique<drake::automotive::TrajectoryFollower<double>>(
           trajectory,
           sampling_time);
   system->set_name(name);
   trajectory_follower_system_ =
       builder->template AddSystem<drake::automotive::TrajectoryFollower<double>>(
           std::move(system));

   /*********************
    * Diagram Wiring
    *********************/
   // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
   // agents, reuse?
   auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
   builder->Connect(trajectory_follower_system_->pose_output(), ports.pose_descriptor);
   builder->Connect(trajectory_follower_system_->velocity_output(), ports.velocity_descriptor);
   car_vis_applicator->AddCarVis(
       std::make_unique<drake::automotive::PriusVis<double>>(id, name));

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

 int Initialize(drake::systems::Context<double>* context) override {
   igndbg << "TrajectoryAgent initialize" << std::endl;

   return 0;
 }

 drake::systems::System<double>* get_system() const { return trajectory_follower_system_; }

private:
 drake::automotive::TrajectoryFollower<double>* trajectory_follower_system_;
};

class TrajectoryAgentFactory final : public delphyne::AgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<double>> Create() {
    return std::make_unique<TrajectoryAgent>();
  }
};

}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::TrajectoryAgentFactory,
                                  delphyne::AgentPluginFactory)

