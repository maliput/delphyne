// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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

#include "systems/unicycle_car.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>
#include <drake/common/drake_assert.h>
#include <drake/systems/framework/vector_base.h>

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
const AngularRateAccelerationCommand<T>& get_input(const UnicycleCar<T>* unicycle_car,
                                                   const drake::systems::Context<T>& context) {
  const AngularRateAccelerationCommand<T>* const input =
      unicycle_car->template EvalVectorInput<AngularRateAccelerationCommand>(context, 0);
  DRAKE_DEMAND(input);
  return *input;
}

}  // namespace

template <typename T>
UnicycleCar<T>::UnicycleCar(const SimpleCarState<T>& initial_context_state) {
  this->DeclareVectorInputPort(AngularRateAccelerationCommand<T>());
  this->DeclareVectorOutputPort(&UnicycleCar::CalcStateOutput);
  this->DeclareVectorOutputPort(&UnicycleCar::CalcPose);
  this->DeclareVectorOutputPort(&UnicycleCar::CalcVelocity);
  this->DeclareContinuousState(initial_context_state);
}

template <typename T>
const drake::systems::OutputPort<T>& UnicycleCar<T>::state_output() const {
  return this->get_output_port(0);
}

template <typename T>
const drake::systems::OutputPort<T>& UnicycleCar<T>::pose_output() const {
  return this->get_output_port(1);
}

template <typename T>
const drake::systems::OutputPort<T>& UnicycleCar<T>::velocity_output() const {
  return this->get_output_port(2);
}

template <typename T>
void UnicycleCar<T>::CalcStateOutput(const drake::systems::Context<T>& context, SimpleCarState<T>* output) const {
  const SimpleCarState<T>& state = get_state(context);
  output->set_value(state.get_value());
}

template <typename T>
void UnicycleCar<T>::CalcPose(const drake::systems::Context<T>& context, PoseVector<T>* pose) const {
  const SimpleCarState<T>& state = get_state(context);
  pose->set_translation(Eigen::Translation<T, 3>(state.x(), state.y(), 0));
  const drake::Vector3<T> z_axis{0.0, 0.0, 1.0};
  const Eigen::AngleAxis<T> rotation(state.heading(), z_axis);
  pose->set_rotation(Eigen::Quaternion<T>(rotation));
}

template <typename T>
void UnicycleCar<T>::CalcVelocity(const drake::systems::Context<T>& context,
                                  drake::systems::rendering::FrameVelocity<T>* velocity) const {
  using std::cos;
  using std::max;
  using std::sin;

  const SimpleCarState<T>& state = get_state(context);

  // Convert the state derivatives into a spatial velocity.
  drake::multibody::SpatialVelocity<T> output;
  output.translational().x() = state.velocity() * cos(state.heading());
  output.translational().y() = state.velocity() * sin(state.heading());
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
void UnicycleCar<T>::DoCalcTimeDerivatives(const drake::systems::Context<T>& context,
                                           drake::systems::ContinuousState<T>* derivatives) const {
  // Obtain the state.
  const SimpleCarState<T>& state = get_state(context);

  // Obtain the input.
  const AngularRateAccelerationCommand<T>* const input =
      this->template EvalVectorInput<AngularRateAccelerationCommand>(context, 0);
  DRAKE_ASSERT(input);

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  drake::systems::VectorBase<T>& vector_derivatives = derivatives->get_mutable_vector();
  SimpleCarState<T>* const rates = dynamic_cast<SimpleCarState<T>*>(&vector_derivatives);
  DRAKE_ASSERT(rates);

  ImplCalcTimeDerivatives(state, *input, rates);
}

template <typename T>
void UnicycleCar<T>::ImplCalcTimeDerivatives(const SimpleCarState<T>& state,
                                             const AngularRateAccelerationCommand<T>& input,
                                             SimpleCarState<T>* rates) const {
  using std::cos;
  using std::sin;

  rates->set_x(state.velocity() * cos(state.heading()));
  rates->set_y(state.velocity() * sin(state.heading()));
  rates->set_heading(input.angular_rate());
  rates->set_velocity(input.acceleration());
}

template class UnicycleCar<double>;

}  // namespace delphyne
