// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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
//
#include "systems/bicycle_car.h"

#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Geometry>
#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>

namespace delphyne {

template <typename T>
BicycleCar<T>::BicycleCar() : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<BicycleCar>{}) {
  auto& steering_input = this->DeclareInputPort(drake::systems::kVectorValued, 1);
  auto& force_input = this->DeclareInputPort(drake::systems::kVectorValued, 1);
  auto& state_output = this->DeclareVectorOutputPort(&BicycleCar::CopyOutState);
  static_assert(BicycleCarStateIndices::kPsi == 0,
                "BicycleCar requires BicycleCarStateIndices::kPsi to be the "
                "0th element.");
  static_assert(BicycleCarStateIndices::kPsiDot == 1,
                "BicycleCar requires BicycleCarStateIndices::kPsiDot to be the "
                "1st element.");
  this->DeclareContinuousState(BicycleCarState<T>(),
                               1,                                             // num_q (Ψ)
                               1,                                             // num_v (Ψ_dot)
                               BicycleCarStateIndices::kNumCoordinates - 2);  // num_z (all but Ψ, Ψ_dot)
  // TODO(jadecastro): Expose translational second-order structure of `sx`, `sy`
  // using `vel` as the generalized velocity (#5323).

  steering_input_port_ = steering_input.get_index();
  force_input_port_ = force_input.get_index();
  state_output_port_ = state_output.get_index();

  this->DeclareNumericParameter(BicycleCarParameters<T>());
}

template <typename T>
template <typename U>
BicycleCar<T>::BicycleCar(const BicycleCar<U>&) : BicycleCar() {}

template <typename T>
BicycleCar<T>::~BicycleCar() {}

template <typename T>
const drake::systems::InputPort<T>& BicycleCar<T>::get_steering_input_port() const {
  return drake::systems::System<T>::get_input_port(steering_input_port_);
}

template <typename T>
const drake::systems::InputPort<T>& BicycleCar<T>::get_force_input_port() const {
  return drake::systems::System<T>::get_input_port(force_input_port_);
}

template <typename T>
const drake::systems::OutputPort<T>& BicycleCar<T>::get_state_output_port() const {
  return drake::systems::System<T>::get_output_port(state_output_port_);
}

template <typename T>
void BicycleCar<T>::CopyOutState(const drake::systems::Context<T>& context, BicycleCarState<T>* output_vector) const {
  // Obtain the state.
  const drake::systems::VectorBase<T>& context_state = context.get_continuous_state_vector();
  const BicycleCarState<T>* const state = dynamic_cast<const BicycleCarState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  output_vector->set_value(state->get_value());
}

// Calculate the continuous-time derivatives.
template <typename T>
void BicycleCar<T>::DoCalcTimeDerivatives(const drake::systems::Context<T>& context,
                                          drake::systems::ContinuousState<T>* derivatives) const {
  // Obtain the parameters, states, inputs, and state derivatives.
  const int kParamsIndex = 0;
  const BicycleCarParameters<T>& params =
      this->template GetNumericParameter<BicycleCarParameters>(context, kParamsIndex);
  const drake::systems::VectorBase<T>& context_state = context.get_continuous_state_vector();
  const BicycleCarState<T>* const state = dynamic_cast<const BicycleCarState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  const drake::systems::BasicVector<T>* steering =
      this->EvalVectorInput(context, get_steering_input_port().get_index());
  DRAKE_ASSERT(steering != nullptr);

  const drake::systems::BasicVector<T>* force = this->EvalVectorInput(context, get_force_input_port().get_index());
  DRAKE_ASSERT(force != nullptr);

  DRAKE_ASSERT(derivatives != nullptr);
  drake::systems::VectorBase<T>& derivative_vector = derivatives->get_mutable_vector();
  BicycleCarState<T>* const state_derivatives = dynamic_cast<BicycleCarState<T>*>(&derivative_vector);
  DRAKE_ASSERT(state_derivatives != nullptr);

  ImplCalcTimeDerivatives(params, *state, *steering, *force, state_derivatives);
}

template <typename T>
void BicycleCar<T>::ImplCalcTimeDerivatives(const BicycleCarParameters<T>& params, const BicycleCarState<T>& state,
                                            const drake::systems::BasicVector<T>& steering,
                                            const drake::systems::BasicVector<T>& force,
                                            BicycleCarState<T>* derivatives) const {
  DRAKE_DEMAND(params.IsValid());

  using std::cos;
  using std::pow;
  using std::sin;

  // Parse and validate the parameters.
  const T m = params.mass();
  const T lr = params.lr();
  const T lf = params.lf();
  const T Iz = params.Iz();
  const T Cf = params.Cf();
  const T Cr = params.Cr();

  DRAKE_DEMAND(m > 0.);
  DRAKE_DEMAND(Iz > 0.);

  // Parse the inputs.
  const T delta = steering[0];
  const T F_in = force[0];

  // Parse and validate the states.
  const T Psi{state.Psi()};
  const T Psi_dot{state.Psi_dot()};
  const T beta{state.beta()};
  const T vel{state.vel()};

  DRAKE_DEMAND(vel != 0.);  // N.B. Protection against the singular solution.
  // TODO(jadecastro): Enable vel = 0. (see #5318).

  const T torsional_stiffness = Cr * lr - Cf * lf;
  const T front_torsional_stiffness = Cf * lf;
  const T torsional_damping = (Cf * pow(lf, 2.) + Cr * pow(lr, 2.)) / vel;

  // Compute the differential equations of motion.
  // clang-format off
  const T Psi_ddot = torsional_stiffness / Iz * beta -
                     torsional_damping / Iz * Psi_dot +
                     front_torsional_stiffness / Iz * delta;
  const T beta_dot = (torsional_stiffness / (m * pow(vel, 2.)) - 1.) * Psi_dot +
                     Cf / (m * vel) * delta - (Cf + Cr) / (m * vel) * beta;
  // clang-format on
  const T vel_dot = F_in / m;  // a_x in Althoff & Dolan, 2014.
  const T sx_dot = vel * cos(beta + Psi);
  const T sy_dot = vel * sin(beta + Psi);

  derivatives->set_Psi(Psi_dot);
  derivatives->set_Psi_dot(Psi_ddot);
  derivatives->set_beta(beta_dot);
  derivatives->set_vel(vel_dot);
  derivatives->set_sx(sx_dot);
  derivatives->set_sy(sy_dot);
}

}  // namespace delphyne

// These instantiations must match the API documentation in bicycle_car.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::delphyne::BicycleCar)
