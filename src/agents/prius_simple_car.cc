// Copyright 2017 Toyota Research Institute

// clalancette: The vast majority of the code below is copied from
// https://github.com/RobotLocomotion/drake/blob/f6f23c5bc539a6aaf754c27b69ef14a69ab3430f/automotive/simple_car.cc
// and
// https://github.com/RobotLocomotion/drake/blob/f6f23c5bc539a6aaf754c27b69ef14a69ab3430f/automotive/simple_car.h
//
// I've modified it slightly to
// 1) Combine the .cc and .h into one file (given that we build this into a .so
//    there is no reason to have the separated file),
// 2) Add in namespaces, and
// 3) Add in the pieces necessary to make it a loadable agent (registering it
//    as a shared library and overriding the appropriate methods).

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <backend/ign_publisher_system.h>
#include <backend/ign_subscriber_system.h>
#include <backend/translation_systems/drake_simple_car_state_to_ign.h>
#include <backend/translation_systems/ign_driving_command_to_drake.h>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/eigen_types.h"
#include "drake/math/saturate.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

#include "../../include/delphyne/agent_plugin_base.h"
#include "../../include/delphyne/linb-any"

namespace delphyne {

namespace {

// Obtain our continuous state from a context.
template <typename T>
const drake::automotive::SimpleCarState<T>& get_state(
    const drake::systems::Context<T>& context) {
  const drake::systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const drake::automotive::SimpleCarState<T>* const state =
      dynamic_cast<const drake::automotive::SimpleCarState<T>*>(&context_state);
  DRAKE_DEMAND(state);
  return *state;
}

// Obtain our parameters from a context.
template <typename T>
const drake::automotive::SimpleCarParams<T>& get_params(
    const drake::systems::Context<T>& context) {
  const drake::automotive::SimpleCarParams<T>* const params =
      dynamic_cast<const drake::automotive::SimpleCarParams<T>*>(
          &context.get_numeric_parameter(0));
  DRAKE_DEMAND(params);
  return *params;
}

}  // namespace

class LoadablePriusSimpleCarDouble final
    : public delphyne::AgentPlugin {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LoadablePriusSimpleCarDouble)

  LoadablePriusSimpleCarDouble() {
    igndbg << "LoadablePriusSimpleCar constructor" << std::endl;
    this->DeclareVectorInputPort(drake::automotive::DrivingCommand<double>());
    this->DeclareVectorOutputPort(
        &LoadablePriusSimpleCarDouble::CalcStateOutput);
    this->DeclareVectorOutputPort(&LoadablePriusSimpleCarDouble::CalcPose);
    this->DeclareVectorOutputPort(&LoadablePriusSimpleCarDouble::CalcVelocity);
    this->DeclareContinuousState(drake::automotive::SimpleCarState<double>());
    this->DeclareNumericParameter(drake::automotive::SimpleCarParams<double>());

    this->DeclareInequalityConstraint(
        &LoadablePriusSimpleCarDouble::CalcSteeringAngleConstraint, 2,
        "steering angle limit");
    this->DeclareInequalityConstraint(
        &LoadablePriusSimpleCarDouble::CalcAccelerationConstraint, 2,
        "acceleration limit");
    this->DeclareInequalityConstraint(
        &LoadablePriusSimpleCarDouble::CalcVelocityConstraint, 2,
        "velocity limit");
  }

  int Configure(const std::map<std::string, linb::any>& parameters,
                drake::systems::DiagramBuilder<double>* builder,
                drake::lcm::DrakeLcmInterface* lcm, const std::string& name,
                int id,
                drake::systems::rendering::PoseAggregator<double>* aggregator,
                drake::automotive::CarVisApplicator<double>* car_vis_applicator)
      override {
    igndbg << "LoadablePriusSimpleCar configure" << std::endl;

    std::string command_channel = "teleop/" + name;
    auto driving_command_subscriber = builder->template AddSystem<
        IgnSubscriberSystem<ignition::msgs::AutomotiveDrivingCommand>>(
        command_channel);

    auto driving_command_translator =
        builder->template AddSystem<IgnDrivingCommandToDrake>();

    // Ignition driving commands received through the subscriber are translated
    // to Drake.
    builder->Connect(*driving_command_subscriber, *driving_command_translator);

    // And then the translated Drake command is sent to the car.
    builder->Connect(*driving_command_translator, *this);

    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
    builder->Connect(this->pose_output(), ports.pose_descriptor);
    builder->Connect(this->velocity_output(), ports.velocity_descriptor);
    car_vis_applicator->AddCarVis(
        std::make_unique<drake::automotive::PriusVis<double>>(id, name));

    auto car_state_translator =
        builder->template AddSystem<DrakeSimpleCarStateToIgn>();

    const std::string car_state_channel =
        "agents/" + std::to_string(id) + "/state";
    auto car_state_publisher = builder->template AddSystem<
        IgnPublisherSystem<ignition::msgs::SimpleCarState>>(car_state_channel);

    // Drake car states are translated to ignition.
    builder->Connect(this->state_output(),
                     car_state_translator->get_input_port(0));

    // And then the translated ingition car state is published.
    builder->Connect(*car_state_translator, *car_state_publisher);

    return 0;
  }

  int Initialize(drake::systems::Context<double>* context) override {
    igndbg << "LoadablePriusSimpleCar initialize" << std::endl;
    return 0;
  }

 private:
  const drake::systems::OutputPort<double>& state_output() const {
    igndbg << "LoadablePriusSimpleCar state_output" << std::endl;
    return this->get_output_port(0);
  }

  const drake::systems::OutputPort<double>& pose_output() const {
    igndbg << "LoadablePriusSimpleCar pose_output" << std::endl;
    return this->get_output_port(1);
  }

  const drake::systems::OutputPort<double>& velocity_output() const {
    igndbg << "LoadablePriusSimpleCar velocity_output" << std::endl;
    return this->get_output_port(2);
  }

  void CalcStateOutput(
      const drake::systems::Context<double>& context,
      drake::automotive::SimpleCarState<double>* output) const {
    const drake::automotive::SimpleCarState<double>& state = get_state(context);
    output->set_value(state.get_value());

    // Don't allow small negative velocities to escape our state.
    using std::max;
    output->set_velocity(max(0.0, state.velocity()));
  }

  void CalcPose(const drake::systems::Context<double>& context,
                drake::systems::rendering::PoseVector<double>* pose) const {
    const drake::automotive::SimpleCarState<double>& state = get_state(context);
    pose->set_translation(
        Eigen::Translation<double, 3>(state.x(), state.y(), 0));
    const drake::Vector3<double> z_axis{0.0, 0.0, 1.0};
    const Eigen::AngleAxis<double> rotation(state.heading(), z_axis);
    pose->set_rotation(Eigen::Quaternion<double>(rotation));
  }

  void CalcVelocity(
      const drake::systems::Context<double>& context,
      drake::systems::rendering::FrameVelocity<double>* velocity) const {
    using std::cos;
    using std::max;
    using std::sin;

    const drake::automotive::SimpleCarState<double>& state = get_state(context);
    const double nonneg_velocity = max(0.0, state.velocity());

    // Convert the state derivatives into a spatial velocity.
    drake::multibody::SpatialVelocity<double> output;
    output.translational().x() = nonneg_velocity * cos(state.heading());
    output.translational().y() = nonneg_velocity * sin(state.heading());
    output.translational().z() = 0.0;
    output.rotational().x() = 0.0;
    output.rotational().y() = 0.0;
    // The rotational velocity around the z-axis is actually rates.heading(),
    // which is a function of the input steering angle. We set it to zero so
    // that
    // this system is not direct-feedthrough.
    output.rotational().z() = 0.0;
    velocity->set_velocity(output);
  }

  void ImplCalcTimeDerivatives(
      const drake::automotive::SimpleCarParams<double>& params,
      const drake::automotive::SimpleCarState<double>& state,
      const drake::automotive::DrivingCommand<double>& input,
      drake::automotive::SimpleCarState<double>* rates) const {
    using std::abs;
    using std::cos;
    using std::max;
    using std::sin;

    // Sanity check our input.
    DRAKE_DEMAND(abs(input.steering_angle()) < M_PI);

    // Compute the smooth acceleration that the vehicle actually executes.
    // TODO(jwnimmer-tri) We should saturate to params.max_acceleration().
    const double desired_acceleration = input.acceleration();
    const double smooth_acceleration =
        drake::automotive::calc_smooth_acceleration(
            desired_acceleration, params.max_velocity(),
            params.velocity_limit_kp(), state.velocity());

    // Determine steering.
    const double saturated_steering_angle = drake::math::saturate(
        input.steering_angle(), -params.max_abs_steering_angle(),
        params.max_abs_steering_angle());
    const double curvature = tan(saturated_steering_angle) / params.wheelbase();

    // Don't allow small negative velocities to affect position or heading.
    const double nonneg_velocity = max(0.0, state.velocity());

    rates->set_x(nonneg_velocity * cos(state.heading()));
    rates->set_y(nonneg_velocity * sin(state.heading()));
    rates->set_heading(curvature * nonneg_velocity);
    rates->set_velocity(smooth_acceleration);
  }

  // System<double> overrides
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    // Obtain the parameters.
    const drake::automotive::SimpleCarParams<double>& params =
        this->template GetNumericParameter<drake::automotive::SimpleCarParams>(
            context, 0);

    // Obtain the state.
    const drake::automotive::SimpleCarState<double>& state = get_state(context);

    // Obtain the input.
    const drake::automotive::DrivingCommand<double>* const input =
        this->template EvalVectorInput<drake::automotive::DrivingCommand>(
            context, 0);
    DRAKE_ASSERT(input);

    // Obtain the result structure.
    DRAKE_ASSERT(derivatives != nullptr);
    drake::systems::VectorBase<double>& vector_derivatives =
        derivatives->get_mutable_vector();
    drake::automotive::SimpleCarState<double>* const rates =
        dynamic_cast<drake::automotive::SimpleCarState<double>*>(
            &vector_derivatives);
    DRAKE_ASSERT(rates);

    ImplCalcTimeDerivatives(params, state, *input, rates);
  }

  // params.max_abs_steering_angle - input.steering_angle ≥ 0.
  // params.max_abs_steering_angle + input.steering_angle ≥ 0.
  void CalcSteeringAngleConstraint(
      const drake::systems::Context<double>& context,
      drake::VectorX<double>* value) const {
    const drake::automotive::DrivingCommand<double>& input =
        *this->template EvalVectorInput<drake::automotive::DrivingCommand>(
            context, 0);
    const drake::automotive::SimpleCarParams<double>& params =
        get_params(context);
    *value = drake::Vector2<double>(
        params.max_abs_steering_angle() - input.steering_angle(),
        params.max_abs_steering_angle() + input.steering_angle());
  }

  // params.max_acceleration - input.acceleration ≥ 0,
  // params.max_acceleration + input.acceleration ≥ 0.
  void CalcAccelerationConstraint(
      const drake::systems::Context<double>& context,
      drake::VectorX<double>* value) const {
    const drake::automotive::DrivingCommand<double>& input =
        *this->template EvalVectorInput<drake::automotive::DrivingCommand>(
            context, 0);
    const drake::automotive::SimpleCarParams<double>& params =
        get_params(context);
    *value = drake::Vector2<double>(
        params.max_acceleration() - input.acceleration(),
        params.max_acceleration() + input.acceleration());
  }

  // params.max_velocity - state.velocity ≥ 0,
  // state.velocity ≥ 0.
  void CalcVelocityConstraint(const drake::systems::Context<double>& context,
                              drake::VectorX<double>* value) const {
    const drake::automotive::SimpleCarState<double>& state = get_state(context);
    const drake::automotive::SimpleCarParams<double>& params =
        get_params(context);
    *value = drake::Vector2<double>(params.max_velocity() - state.velocity(),
                                    state.velocity());
  }

  const drake::automotive::DrivingCommandTranslator driving_command_translator_;
  const drake::automotive::SimpleCarStateTranslator translator_;
};

class LoadablePriusSimpleCarFactoryDouble final
    : public delphyne::AgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<double>> Create() {
    return std::make_unique<LoadablePriusSimpleCarDouble>();
  }
};

}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::LoadablePriusSimpleCarFactoryDouble,
                                  delphyne::AgentPluginFactory)
