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


//class TrajectoryCar final : public delphyne::AgentPlugin {
// public:
//  typedef typename drake::automotive::Curve2<double>::Point2T Point2;
//
//  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryCar)
//
//  /// Constructs a TrajectoryCar system that traces a given two-dimensional @p
//  /// curve.  Throws an error if the curve is empty (has a zero @p path_length).
//  TrajectoryCar() : curve_(nullptr) {
//    this->DeclareInputPort(drake::systems::kVectorValued,
//                           1 /* single-valued input */);
//    this->DeclareVectorOutputPort(&TrajectoryCar::CalcStateOutput);
//    this->DeclareVectorOutputPort(&TrajectoryCar::CalcPoseOutput);
//    this->DeclareVectorOutputPort(&TrajectoryCar::CalcVelocityOutput);
//    this->DeclareContinuousState(
//        drake::automotive::TrajectoryCarState<double>());
//    this->DeclareNumericParameter(
//        drake::automotive::TrajectoryCarParams<double>());
//  }
//
//  int Configure(const std::map<std::string, linb::any>& parameters,
//                drake::systems::DiagramBuilder<double>* builder,
//                drake::lcm::DrakeLcmInterface* lcm, const std::string& name,
//                int id,
//                drake::systems::rendering::PoseAggregator<double>* aggregator,
//                drake::automotive::CarVisApplicator<double>* car_vis_applicator)
//      override {
//    igndbg << "LoadablePriusTrajectoryCar configure" << std::endl;
//    curve_ = new drake::automotive::Curve2<double>(
//        linb::any_cast<drake::automotive::Curve2<double>>(
//            parameters.at("curve")));
//
//    if (curve_->path_length() == 0.0) {
//      ignerr << "Invalid curve passed to TrajectoryCar" << std::endl;
//      return -1;
//    }
//
//    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
//    builder->Connect(this->pose_output(), ports.pose_descriptor);
//    builder->Connect(this->velocity_output(), ports.velocity_descriptor);
//    car_vis_applicator->AddCarVis(
//        std::make_unique<drake::automotive::PriusVis<double>>(id, name));
//
//    auto car_state_translator = builder->AddSystem<DrakeSimpleCarStateToIgn>();
//
//    const std::string agent_state_channel =
//        "agents/" + std::to_string(id) + "/state";
//    auto agent_state_publisher =
//        builder->AddSystem<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
//            agent_state_channel);
//
//    // The Drake car state is translated to ignition.
//    builder->Connect(this->raw_pose_output(),
//                     car_state_translator->get_input_port(0));
//
//    // And then the translated ignition message is published.
//    builder->Connect(*car_state_translator, *agent_state_publisher);
//
//    return 0;
//  }
//
//  ~TrajectoryCar() {
//    delete curve_;
//    curve_ = nullptr;
//  }
//
//  int Initialize(drake::systems::Context<double>* context) override {
//    igndbg << "LoadablePriusTrajectoryCar initialize" << std::endl;
//    return 0;
//  }
//
// private:
//  /// The command input port (optional).
//  const drake::systems::InputPortDescriptor<double>& command_input() const {
//    igndbg << "LoadablePriusTrajectoryCar command_input" << std::endl;
//    return this->get_input_port(0);
//  }
//  /// See class description for details about the following ports.
//  /// @{
//  const drake::systems::OutputPort<double>& raw_pose_output() const {
//    igndbg << "LoadablePriusTrajectoryCar raw_pose_output" << std::endl;
//    return this->get_output_port(0);
//  }
//  const drake::systems::OutputPort<double>& pose_output() const {
//    igndbg << "LoadablePriusTrajectoryCar pose_output" << std::endl;
//    return this->get_output_port(1);
//  }
//  const drake::systems::OutputPort<double>& velocity_output() const {
//    igndbg << "LoadablePriusTrajectoryCar velocity_output" << std::endl;
//    return this->get_output_port(2);
//  }
//  /// @}
//
//  /// Data structure returned by CalcRawPose containing raw pose information.
//  struct PositionHeading {
//    Point2 position = Point2::Zero();
//    double heading{0.};
//  };
//
//  void CalcStateOutput(
//      const drake::systems::Context<double>& context,
//      drake::automotive::SimpleCarState<double>* output_vector) const {
//    const drake::automotive::TrajectoryCarState<double>& state =
//        GetState(context);
//    const auto raw_pose = CalcRawPose(state);
//    ImplCalcOutput(raw_pose, state, output_vector);
//  }
//
//  void CalcPoseOutput(
//      const drake::systems::Context<double>& context,
//      drake::systems::rendering::PoseVector<double>* pose) const {
//    const auto raw_pose = CalcRawPose(GetState(context));
//    ImplCalcPose(raw_pose, pose);
//  }
//
//  void CalcVelocityOutput(
//      const drake::systems::Context<double>& context,
//      drake::systems::rendering::FrameVelocity<double>* velocity) const {
//    const drake::automotive::TrajectoryCarState<double>& state =
//        GetState(context);
//    const auto raw_pose = CalcRawPose(state);
//    ImplCalcVelocity(raw_pose, state, velocity);
//  }
//
//  void DoCalcTimeDerivatives(
//      const drake::systems::Context<double>& context,
//      drake::systems::ContinuousState<double>* derivatives) const override {
//    // Obtain the parameters.
//    const drake::automotive::TrajectoryCarParams<double>& params =
//        this->template GetNumericParameter<
//            drake::automotive::TrajectoryCarParams>(context, 0);
//
//    // Obtain the state.
//    const drake::automotive::TrajectoryCarState<double>* const state =
//        dynamic_cast<const drake::automotive::TrajectoryCarState<double>*>(
//            &context.get_continuous_state_vector());
//    DELPHYNE_ASSERT(state);
//
//    // Obtain the input.
//    const drake::systems::BasicVector<double>* input =
//        this->template EvalVectorInput<drake::systems::BasicVector>(context, 0);
//
//    // If the input is null, then apply a default acceleration of zero.
//    const auto default_input = drake::systems::BasicVector<double>::Make(0.);
//    if (input == nullptr) {
//      input = default_input.get();
//    }
//    DELPHYNE_ASSERT(input->size() == 1);  // Expect the input to have only a
//                                          // single acceleration value.
//
//    // Obtain the result structure.
//    DELPHYNE_ASSERT(derivatives != nullptr);
//    drake::systems::VectorBase<double>& vector_derivatives =
//        derivatives->get_mutable_vector();
//    drake::automotive::TrajectoryCarState<double>* const rates =
//        dynamic_cast<drake::automotive::TrajectoryCarState<double>*>(
//            &vector_derivatives);
//    DELPHYNE_ASSERT(rates);
//
//    ImplCalcTimeDerivatives(params, *state, *input, rates);
//  }
//
//  void ImplCalcOutput(
//      const PositionHeading& raw_pose,
//      const drake::automotive::TrajectoryCarState<double>& state,
//      drake::automotive::SimpleCarState<double>* output) const {
//    // Convert raw pose to output type.
//    output->set_x(raw_pose.position[0]);
//    output->set_y(raw_pose.position[1]);
//    output->set_heading(raw_pose.heading);
//    output->set_velocity(state.speed());
//  }
//
//  void ImplCalcPose(const PositionHeading& raw_pose,
//                    drake::systems::rendering::PoseVector<double>* pose) const {
//    // Convert the raw pose into a pose vector.
//    pose->set_translation(Eigen::Translation<double, 3>(
//        raw_pose.position[0], raw_pose.position[1], 0));
//    const drake::Vector3<double> z_axis{0.0, 0.0, 1.0};
//    const Eigen::AngleAxis<double> rotation(raw_pose.heading, z_axis);
//    pose->set_rotation(Eigen::Quaternion<double>(rotation));
//  }
//
//  void ImplCalcVelocity(
//      const PositionHeading& raw_pose,
//      const drake::automotive::TrajectoryCarState<double>& state,
//      drake::systems::rendering::FrameVelocity<double>* velocity) const {
//    using std::cos;
//    using std::sin;
//
//    // Convert the state derivatives into a spatial velocity.
//    drake::multibody::SpatialVelocity<double> output;
//    output.translational().x() = state.speed() * cos(raw_pose.heading);
//    output.translational().y() = state.speed() * sin(raw_pose.heading);
//    output.translational().z() = 0.0;
//    output.rotational().x() = 0.0;
//    output.rotational().y() = 0.0;
//    // N.B. The instantaneous rotation rate is always zero, as the Curve2 is
//    // constructed from line segments.
//    output.rotational().z() = 0.0;
//    velocity->set_velocity(output);
//  }
//
//  void ImplCalcTimeDerivatives(
//      const drake::automotive::TrajectoryCarParams<double>& params,
//      const drake::automotive::TrajectoryCarState<double>& state,
//      const drake::systems::BasicVector<double>& input,
//      drake::automotive::TrajectoryCarState<double>* rates) const {
//    using std::max;
//
//    // Create an acceleration profile that caps the maximum speed of the vehicle
//    // as it approaches or exceeds the `params.max_speed()` limit, passing the
//    // input acceleration through when away from the limit.  Note that
//    // accelerations of zero are passed through unaffected.
//    const double desired_acceleration = input.GetAtIndex(0);
//    const double smooth_acceleration =
//        drake::automotive::calc_smooth_acceleration(
//            desired_acceleration, params.max_speed(), params.speed_limit_kp(),
//            state.speed());
//
//    // Don't allow small negative velocities to affect position.
//    const double nonneg_velocity = max(0.0, state.speed());
//
//    rates->set_position(nonneg_velocity);
//    rates->set_speed(smooth_acceleration);
//  }
//
//  // Extract the appropriately-typed state from the context.
//  const drake::automotive::TrajectoryCarState<double>& GetState(
//      const drake::systems::Context<double>& context) const {
//    auto state =
//        dynamic_cast<const drake::automotive::TrajectoryCarState<double>*>(
//            &context.get_continuous_state_vector());
//    DELPHYNE_DEMAND(state != nullptr);
//    return *state;
//  }
//
//  // Computes the PositionHeading of the trajectory car based on the car's
//  // current position along the curve.
//  const PositionHeading CalcRawPose(
//      const drake::automotive::TrajectoryCarState<double>& state) const {
//    using std::atan2;
//
//    PositionHeading result;
//
//    // Compute the curve at the current longitudinal (along-curve) position.
//    const typename drake::automotive::Curve2<double>::PositionResult pose =
//        curve_->GetPosition(state.position());
//    // TODO(jadecastro): Now that the curve is a function of position rather
//    // than time, we are not acting on a `trajectory` anymore.  Rename this
//    // System to PathFollowingCar or something similar.
//    DELPHYNE_ASSERT(pose.position_dot.norm() > 0.0);
//
//    result.position = pose.position;
//    result.heading = atan2(pose.position_dot[1], pose.position_dot[0]);
//    return result;
//  }
//
//  const drake::automotive::Curve2<double>* curve_;
//};
//
//class LoadablePriusTrajectoryCarFactoryDouble final
//    : public delphyne::AgentPluginFactory {
// public:
//  std::unique_ptr<delphyne::AgentPluginBase<double>> Create() {
//    return std::make_unique<TrajectoryCar>();
//  }
//};

class TrajectoryAgentFactory final : public delphyne::AgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<double>> Create() {
    return std::make_unique<TrajectoryAgent>();
  }
};

}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::TrajectoryAgentFactory,
                                  delphyne::AgentPluginFactory)

