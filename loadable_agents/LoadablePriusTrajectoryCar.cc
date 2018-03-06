// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.

// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:

// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// clalancette: The vast majority of the code below is copied from
// https://github.com/RobotLocomotion/drake/blob/f6f23c5bc539a6aaf754c27b69ef14a69ab3430f/automotive/trajectory_car.cc
// and
// https://github.com/RobotLocomotion/drake/blob/f6f23c5bc539a6aaf754c27b69ef14a69ab3430f/automotive/trajectory_car.h
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

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/automotive/curve2.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/gen/trajectory_car_params.h"
#include "drake/automotive/gen/trajectory_car_state.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/eigen_types.h"
#include "drake/math/saturate.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

#include <backend/agent_plugin_base.h>
#include <backend/ign_subscriber_system.h>
#include <backend/linb-any>
#include <backend/simple_car_state_to_ignition_message_converter.h>

namespace delphyne {
namespace backend {

/// TrajectoryCar models a car that follows a pre-established trajectory.  Note
/// that TrajectoryCar can move forward (up to a given "soft" speed limit) but
/// cannot travel in reverse.
///
/// parameters:
/// * uses systems::Parameters wrapping a TrajectoryCarParams
///
/// state vector:
/// * A TrajectoryCarState, consisting of a position and speed along the given
///   curve, provided as the constructor parameter.
///
/// input vector:
/// * desired acceleration, a systems::BasicVector of size 1 (optional input).
///   If left unconnected, the trajectory car will travel at a constant speed
///   specified in the TrajectoryCarState.
///
/// output port 0:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///   (OutputPort getter: raw_pose_output())
///
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
///   (OutputPort getter: pose_output())
///
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///   (OutputPort getter: velocity_output())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
class LoadablePriusTrajectoryCarDouble final
    : public delphyne::backend::AgentPluginDoubleBase {
 public:
  typedef typename drake::automotive::Curve2<double>::Point2T Point2;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LoadablePriusTrajectoryCarDouble)

  /// Constructs a TrajectoryCar system that traces a given two-dimensional @p
  /// curve.  Throws an error if the curve is empty (has a zero @p path_length).
  LoadablePriusTrajectoryCarDouble() : curve_(nullptr) {
    this->DeclareInputPort(drake::systems::kVectorValued,
                           1 /* single-valued input */);
    this->DeclareVectorOutputPort(
        &LoadablePriusTrajectoryCarDouble::CalcStateOutput);
    this->DeclareVectorOutputPort(
        &LoadablePriusTrajectoryCarDouble::CalcPoseOutput);
    this->DeclareVectorOutputPort(
        &LoadablePriusTrajectoryCarDouble::CalcVelocityOutput);
    this->DeclareContinuousState(
        drake::automotive::TrajectoryCarState<double>());
    this->DeclareNumericParameter(
        drake::automotive::TrajectoryCarParams<double>());
  }

  int Configure(const std::map<std::string, linb::any>& parameters,
                drake::systems::DiagramBuilder<double>* builder,
                drake::lcm::DrakeLcmInterface* lcm, const std::string& name,
                int id,
                drake::systems::rendering::PoseAggregator<double>* aggregator,
                drake::automotive::CarVisApplicator<double>* car_vis_applicator)
      override {
    igndbg << "LoadablePriusTrajectoryCar configure" << std::endl;
    curve_ = new drake::automotive::Curve2<double>(
        linb::any_cast<drake::automotive::Curve2<double>>(
            parameters.at("curve")));

    if (curve_->path_length() == 0.0) {
      ignerr << "Invalid curve passed to TrajectoryCar" << std::endl;
      return -1;
    }

    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
    builder->Connect(this->pose_output(), ports.first);
    builder->Connect(this->velocity_output(), ports.second);
    car_vis_applicator->AddCarVis(
        std::make_unique<drake::automotive::PriusVis<double>>(id, name));

    const std::string channel = std::to_string(id) + "_SIMPLE_CAR_STATE";
    auto pub_converter =
        std::make_unique<SimpleCarStateToIgnitionMessageConverter>();
    auto publisher = builder->AddSystem(
        std::make_unique<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
            channel, std::move(pub_converter)));
    builder->Connect(this->raw_pose_output(), publisher->get_input_port(0));

    return 0;
  }

  ~LoadablePriusTrajectoryCarDouble() {
    delete curve_;
    curve_ = nullptr;
  }

  int Initialize(drake::systems::Context<double>* context) override {
    igndbg << "LoadablePriusTrajectoryCar initialize" << std::endl;
    return 0;
  }

 private:
  /// The command input port (optional).
  const drake::systems::InputPortDescriptor<double>& command_input() const {
    igndbg << "LoadablePriusTrajectoryCar command_input" << std::endl;
    return this->get_input_port(0);
  }
  /// See class description for details about the following ports.
  /// @{
  const drake::systems::OutputPort<double>& raw_pose_output() const {
    igndbg << "LoadablePriusTrajectoryCar raw_pose_output" << std::endl;
    return this->get_output_port(0);
  }
  const drake::systems::OutputPort<double>& pose_output() const {
    igndbg << "LoadablePriusTrajectoryCar pose_output" << std::endl;
    return this->get_output_port(1);
  }
  const drake::systems::OutputPort<double>& velocity_output() const {
    igndbg << "LoadablePriusTrajectoryCar velocity_output" << std::endl;
    return this->get_output_port(2);
  }
  /// @}

  /// Data structure returned by CalcRawPose containing raw pose information.
  struct PositionHeading {
    Point2 position = Point2::Zero();
    double heading{0.};
  };

  void CalcStateOutput(
      const drake::systems::Context<double>& context,
      drake::automotive::SimpleCarState<double>* output_vector) const {
    const drake::automotive::TrajectoryCarState<double>& state =
        GetState(context);
    const auto raw_pose = CalcRawPose(state);
    ImplCalcOutput(raw_pose, state, output_vector);
  }

  void CalcPoseOutput(
      const drake::systems::Context<double>& context,
      drake::systems::rendering::PoseVector<double>* pose) const {
    const auto raw_pose = CalcRawPose(GetState(context));
    ImplCalcPose(raw_pose, pose);
  }

  void CalcVelocityOutput(
      const drake::systems::Context<double>& context,
      drake::systems::rendering::FrameVelocity<double>* velocity) const {
    const drake::automotive::TrajectoryCarState<double>& state =
        GetState(context);
    const auto raw_pose = CalcRawPose(state);
    ImplCalcVelocity(raw_pose, state, velocity);
  }

  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    // Obtain the parameters.
    const drake::automotive::TrajectoryCarParams<double>& params =
        this->template GetNumericParameter<
            drake::automotive::TrajectoryCarParams>(context, 0);

    // Obtain the state.
    const drake::automotive::TrajectoryCarState<double>* const state =
        dynamic_cast<const drake::automotive::TrajectoryCarState<double>*>(
            &context.get_continuous_state_vector());
    DELPHYNE_ASSERT(state);

    // Obtain the input.
    const drake::systems::BasicVector<double>* input =
        this->template EvalVectorInput<drake::systems::BasicVector>(context, 0);

    // If the input is null, then apply a default acceleration of zero.
    const auto default_input = drake::systems::BasicVector<double>::Make(0.);
    if (input == nullptr) {
      input = default_input.get();
    }
    DELPHYNE_ASSERT(input->size() == 1);  // Expect the input to have only a single
                                       // acceleration value.

    // Obtain the result structure.
    DELPHYNE_ASSERT(derivatives != nullptr);
    drake::systems::VectorBase<double>& vector_derivatives =
        derivatives->get_mutable_vector();
    drake::automotive::TrajectoryCarState<double>* const rates =
        dynamic_cast<drake::automotive::TrajectoryCarState<double>*>(
            &vector_derivatives);
    DELPHYNE_ASSERT(rates);

    ImplCalcTimeDerivatives(params, *state, *input, rates);
  }

  void ImplCalcOutput(
      const PositionHeading& raw_pose,
      const drake::automotive::TrajectoryCarState<double>& state,
      drake::automotive::SimpleCarState<double>* output) const {
    // Convert raw pose to output type.
    output->set_x(raw_pose.position[0]);
    output->set_y(raw_pose.position[1]);
    output->set_heading(raw_pose.heading);
    output->set_velocity(state.speed());
  }

  void ImplCalcPose(const PositionHeading& raw_pose,
                    drake::systems::rendering::PoseVector<double>* pose) const {
    // Convert the raw pose into a pose vector.
    pose->set_translation(Eigen::Translation<double, 3>(
        raw_pose.position[0], raw_pose.position[1], 0));
    const drake::Vector3<double> z_axis{0.0, 0.0, 1.0};
    const Eigen::AngleAxis<double> rotation(raw_pose.heading, z_axis);
    pose->set_rotation(Eigen::Quaternion<double>(rotation));
  }

  void ImplCalcVelocity(
      const PositionHeading& raw_pose,
      const drake::automotive::TrajectoryCarState<double>& state,
      drake::systems::rendering::FrameVelocity<double>* velocity) const {
    using std::cos;
    using std::sin;

    // Convert the state derivatives into a spatial velocity.
    drake::multibody::SpatialVelocity<double> output;
    output.translational().x() = state.speed() * cos(raw_pose.heading);
    output.translational().y() = state.speed() * sin(raw_pose.heading);
    output.translational().z() = 0.0;
    output.rotational().x() = 0.0;
    output.rotational().y() = 0.0;
    // N.B. The instantaneous rotation rate is always zero, as the Curve2 is
    // constructed from line segments.
    output.rotational().z() = 0.0;
    velocity->set_velocity(output);
  }

  void ImplCalcTimeDerivatives(
      const drake::automotive::TrajectoryCarParams<double>& params,
      const drake::automotive::TrajectoryCarState<double>& state,
      const drake::systems::BasicVector<double>& input,
      drake::automotive::TrajectoryCarState<double>* rates) const {
    using std::max;

    // Create an acceleration profile that caps the maximum speed of the vehicle
    // as it approaches or exceeds the `params.max_speed()` limit, passing the
    // input acceleration through when away from the limit.  Note that
    // accelerations of zero are passed through unaffected.
    const double desired_acceleration = input.GetAtIndex(0);
    const double smooth_acceleration =
        drake::automotive::calc_smooth_acceleration(
            desired_acceleration, params.max_speed(), params.speed_limit_kp(),
            state.speed());

    // Don't allow small negative velocities to affect position.
    const double nonneg_velocity = max(0.0, state.speed());

    rates->set_position(nonneg_velocity);
    rates->set_speed(smooth_acceleration);
  }

  // Extract the appropriately-typed state from the context.
  const drake::automotive::TrajectoryCarState<double>& GetState(
      const drake::systems::Context<double>& context) const {
    auto state =
        dynamic_cast<const drake::automotive::TrajectoryCarState<double>*>(
            &context.get_continuous_state_vector());
    DELPHYNE_DEMAND(state != nullptr);
    return *state;
  }

  // Computes the PositionHeading of the trajectory car based on the car's
  // current position along the curve.
  const PositionHeading CalcRawPose(
      const drake::automotive::TrajectoryCarState<double>& state) const {
    using std::atan2;

    PositionHeading result;

    // Compute the curve at the current longitudinal (along-curve) position.
    const typename drake::automotive::Curve2<double>::PositionResult pose =
        curve_->GetPosition(state.position());
    // TODO(jadecastro): Now that the curve is a function of position rather
    // than time, we are not acting on a `trajectory` anymore.  Rename this
    // System to PathFollowingCar or something similar.
    DELPHYNE_ASSERT(pose.position_dot.norm() > 0.0);

    result.position = pose.position;
    result.heading = atan2(pose.position_dot[1], pose.position_dot[0]);
    return result;
  }

  const drake::automotive::Curve2<double>* curve_;
  const drake::automotive::SimpleCarStateTranslator translator_;
};

class LoadablePriusTrajectoryCarFactoryDouble final
    : public delphyne::backend::AgentPluginFactoryDoubleBase {
 public:
  std::unique_ptr<delphyne::backend::AgentPluginBase<double>> Create() {
    return std::make_unique<LoadablePriusTrajectoryCarDouble>();
  }
};

}  // namespace backend
}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    delphyne::backend::LoadablePriusTrajectoryCarFactoryDouble,
    delphyne::backend::AgentPluginFactoryDoubleBase)
