// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "systems/simple_car.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Geometry>
#include <drake/common/cond.h>
#include <drake/common/default_scalars.h>
#include <drake/common/double_overloads.h>
#include <drake/common/drake_assert.h>
#include <drake/math/saturate.h>
#include <drake/systems/framework/vector_base.h>

#include "systems/calc_smooth_acceleration.h"

namespace delphyne {

using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;

namespace {  // Local helper function.

// Obtain our continuous state from a context.
template <typename T>
const SimpleCarState<T>& get_state(const drake::systems::Context<T>& context) {
  const drake::systems::VectorBase<T>& context_state = context.get_continuous_state_vector();
  const SimpleCarState<T>* const state = dynamic_cast<const SimpleCarState<T>*>(&context_state);
  DRAKE_DEMAND(state);
  return *state;
}

// Obtain our input from a context.
template <typename T>
const DrivingCommand<T>& get_input(const SimpleCar2<T>* simple_car, const drake::systems::Context<T>& context) {
  const DrivingCommand<T>* const input = simple_car->template EvalVectorInput<DrivingCommand>(context, 0);
  DRAKE_DEMAND(input);
  return *input;
}

// Obtain our parameters from a context.
template <typename T>
const SimpleCarParams<T>& get_params(const drake::systems::Context<T>& context) {
  const SimpleCarParams<T>& params = dynamic_cast<const SimpleCarParams<T>&>(context.get_numeric_parameter(0));
  return params;
}

}  // namespace

template <typename T>
SimpleCar2<T>::SimpleCar2(const SimpleCarState<T>& initial_context_state,
                          const SimpleCarParams<T>& initial_context_parameters)
    : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<SimpleCar2>{}) {
  this->DeclareVectorInputPort(DrivingCommand<T>());
  this->DeclareVectorOutputPort(&SimpleCar2::CalcStateOutput);
  this->DeclareVectorOutputPort(&SimpleCar2::CalcPose);
  this->DeclareVectorOutputPort(&SimpleCar2::CalcVelocity);
  this->DeclareContinuousState(initial_context_state);
  this->DeclareNumericParameter(initial_context_parameters);

  this->DeclareInequalityConstraint(&SimpleCar2::CalcSteeringAngleConstraint, {Eigen::Vector2d::Zero(), std::nullopt},
                                    "steering angle limit");
  this->DeclareInequalityConstraint(&SimpleCar2::CalcAccelerationConstraint, {Eigen::Vector2d::Zero(), std::nullopt},
                                    "acceleration limit");
  this->DeclareInequalityConstraint(&SimpleCar2::CalcVelocityConstraint, {Eigen::Vector2d::Zero(), std::nullopt},
                                    "velocity limit");
}

template <typename T>
template <typename U>
SimpleCar2<T>::SimpleCar2(const SimpleCar2<U>&) : SimpleCar2() {}

template <typename T>
const drake::systems::OutputPort<T>& SimpleCar2<T>::state_output() const {
  return this->get_output_port(0);
}

template <typename T>
const drake::systems::OutputPort<T>& SimpleCar2<T>::pose_output() const {
  return this->get_output_port(1);
}

template <typename T>
const drake::systems::OutputPort<T>& SimpleCar2<T>::velocity_output() const {
  return this->get_output_port(2);
}

template <typename T>
void SimpleCar2<T>::CalcStateOutput(const drake::systems::Context<T>& context, SimpleCarState<T>* output) const {
  const SimpleCarState<T>& state = get_state(context);
  output->set_value(state.get_value());

  // Don't allow small negative velocities to escape our state.
  using std::max;
  output->set_velocity(max(T(0), state.velocity()));
}

template <typename T>
void SimpleCar2<T>::CalcPose(const drake::systems::Context<T>& context, PoseVector<T>* pose) const {
  const SimpleCarState<T>& state = get_state(context);
  pose->set_translation(Eigen::Translation<T, 3>(state.x(), state.y(), 0));
  const drake::Vector3<T> z_axis{0.0, 0.0, 1.0};
  const Eigen::AngleAxis<T> rotation(state.heading(), z_axis);
  pose->set_rotation(Eigen::Quaternion<T>(rotation));
}

template <typename T>
void SimpleCar2<T>::CalcVelocity(const drake::systems::Context<T>& context,
                                 drake::systems::rendering::FrameVelocity<T>* velocity) const {
  using std::cos;
  using std::max;
  using std::sin;

  const SimpleCarState<T>& state = get_state(context);
  const T nonneg_velocity = max(T(0), state.velocity());

  // Convert the state derivatives into a spatial velocity.
  drake::multibody::SpatialVelocity<T> output;
  output.translational().x() = nonneg_velocity * cos(state.heading());
  output.translational().y() = nonneg_velocity * sin(state.heading());
  output.translational().z() = T(0);
  output.rotational().x() = T(0);
  output.rotational().y() = T(0);
  // The rotational velocity around the z-axis is actually rates.heading(),
  // which is a function of the input steering angle. We set it to zero so that
  // this system is not direct-feedthrough.
  output.rotational().z() = T(0);
  velocity->set_velocity(output);
}

template <typename T>
void SimpleCar2<T>::DoCalcTimeDerivatives(const drake::systems::Context<T>& context,
                                          drake::systems::ContinuousState<T>* derivatives) const {
  // Obtain the parameters.
  const SimpleCarParams<T>& params = this->template GetNumericParameter<SimpleCarParams>(context, 0);

  // Obtain the state.
  const SimpleCarState<T>& state = get_state(context);

  // Obtain the input.
  const DrivingCommand<T>* const input = this->template EvalVectorInput<DrivingCommand>(context, 0);
  DRAKE_ASSERT(input);

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  drake::systems::VectorBase<T>& vector_derivatives = derivatives->get_mutable_vector();
  SimpleCarState<T>* const rates = dynamic_cast<SimpleCarState<T>*>(&vector_derivatives);
  DRAKE_ASSERT(rates);

  ImplCalcTimeDerivatives(params, state, *input, rates);
}

template <typename T>
void SimpleCar2<T>::ImplCalcTimeDerivatives(const SimpleCarParams<T>& params, const SimpleCarState<T>& state,
                                            const DrivingCommand<T>& input, SimpleCarState<T>* rates) const {
  using std::abs;
  using std::cos;
  using std::max;
  using std::sin;

  // Sanity check our input.
  DRAKE_DEMAND(abs(input.steering_angle()) < M_PI);

  // Compute the smooth acceleration that the vehicle actually executes.
  // TODO(jwnimmer-tri) We should saturate to params.max_acceleration().
  const T desired_acceleration = input.acceleration();
  const T smooth_acceleration = calc_smooth_acceleration(desired_acceleration, params.max_velocity(),
                                                         params.velocity_limit_kp(), state.velocity());

  // Determine steering.
  const T saturated_steering_angle =
      drake::math::saturate(input.steering_angle(), -params.max_abs_steering_angle(), params.max_abs_steering_angle());
  const T curvature = tan(saturated_steering_angle) / params.wheelbase();

  // Don't allow small negative velocities to affect position or heading.
  const T nonneg_velocity = max(T(0), state.velocity());

  rates->set_x(nonneg_velocity * cos(state.heading()));
  rates->set_y(nonneg_velocity * sin(state.heading()));
  rates->set_heading(curvature * nonneg_velocity);
  rates->set_velocity(smooth_acceleration);
}

// params.max_abs_steering_angle - input.steering_angle ≥ 0.
// params.max_abs_steering_angle + input.steering_angle ≥ 0.
template <typename T>
void SimpleCar2<T>::CalcSteeringAngleConstraint(const drake::systems::Context<T>& context,
                                                drake::VectorX<T>* value) const {
  const DrivingCommand<T>& input = get_input(this, context);
  const SimpleCarParams<T>& params = get_params(context);
  *value = drake::Vector2<T>(params.max_abs_steering_angle() - input.steering_angle(),
                             params.max_abs_steering_angle() + input.steering_angle());
}

// params.max_acceleration - input.acceleration ≥ 0,
// params.max_acceleration + input.acceleration ≥ 0.
template <typename T>
void SimpleCar2<T>::CalcAccelerationConstraint(const drake::systems::Context<T>& context,
                                               drake::VectorX<T>* value) const {
  const DrivingCommand<T>& input = get_input(this, context);
  const SimpleCarParams<T>& params = get_params(context);
  *value = drake::Vector2<T>(params.max_acceleration() - input.acceleration(),
                             params.max_acceleration() + input.acceleration());
}

// params.max_velocity - state.velocity ≥ 0,
// state.velocity ≥ 0.
template <typename T>
void SimpleCar2<T>::CalcVelocityConstraint(const drake::systems::Context<T>& context, drake::VectorX<T>* value) const {
  const SimpleCarState<T>& state = get_state(context);
  const SimpleCarParams<T>& params = get_params(context);
  *value = drake::Vector2<T>(params.max_velocity() - state.velocity(), state.velocity());
}

}  // namespace delphyne

// These instantiations must match the API documentation in simple_car.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::delphyne::SimpleCar2)
