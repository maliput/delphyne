// Copyright 2020 Toyota Research Institute

#include "systems/unicycle_car.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <drake/common/cond.h>
#include <drake/common/default_scalars.h>
#include <drake/common/double_overloads.h>
#include <drake/common/drake_assert.h>
#include <drake/math/saturate.h>
#include <drake/systems/framework/vector_base.h>

#include <Eigen/Geometry>

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
const AngularRateAccelerationCommand<T>& get_input(const UnicycleCar<T>* unicycle_car,
                                                   const drake::systems::Context<T>& context) {
  const AngularRateAccelerationCommand<T>* const input =
      unicycle_car->template EvalVectorInput<AngularRateAccelerationCommand>(context, 0);
  DRAKE_DEMAND(input);
  return *input;
}

}  // namespace

template <typename T>
UnicycleCar<T>::UnicycleCar(const SimpleCarState<T>& initial_context_state)
    : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<UnicycleCar>{}) {
  this->DeclareVectorInputPort(AngularRateAccelerationCommand<T>());
  this->DeclareVectorOutputPort(&UnicycleCar::CalcStateOutput);
  this->DeclareVectorOutputPort(&UnicycleCar::CalcPose);
  this->DeclareVectorOutputPort(&UnicycleCar::CalcVelocity);
  this->DeclareContinuousState(initial_context_state);
}

template <typename T>
template <typename U>
UnicycleCar<T>::UnicycleCar(const UnicycleCar<U>&) : UnicycleCar() {}

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

}  // namespace delphyne

// These instantiations must match the API documentation in unicycle_car.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::delphyne::UnicycleCar)
