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

#include "systems/dynamic_bicycle_car.h"

#include <algorithm>
#include <cmath>

#include <drake/common/cond.h>
#include <drake/common/default_scalars.h>

namespace delphyne {

template <typename T>
DynamicBicycleCar<T>::DynamicBicycleCar()
    : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<DynamicBicycleCar>{}) {
  this->DeclareVectorInputPort(DynamicBicycleCarInput<T>());
  this->DeclareVectorOutputPort(DynamicBicycleCarState<T>(), &DynamicBicycleCar::CopyStateOut, {this->xc_ticket()});

  // Declares that this system has a continuous state of size
  // DynamicBicycleCarState.size() and in a vector cloned from
  // DynamicBicycleCarState.
  this->DeclareContinuousState(DynamicBicycleCarState<T>());

  // Declares the system's numeric parameters from the named vector
  // dynamic_bicycle_car_params.named_vector.
  this->DeclareNumericParameter(DynamicBicycleCarParams<T>());
}

template <typename T>
const drake::systems::OutputPort<T>& DynamicBicycleCar<T>::get_output_port() const {
  return drake::systems::System<T>::get_output_port(0);
}

template <typename T>
const drake::systems::InputPort<T>& DynamicBicycleCar<T>::get_input_port() const {
  return drake::systems::System<T>::get_input_port(0);
}

template <typename T>
const DynamicBicycleCarState<T>& DynamicBicycleCar<T>::get_state(const drake::systems::Context<T>& context) const {
  const drake::systems::ContinuousState<T>& cstate = context.get_continuous_state();
  // Casts the continuous state vector from a VectorBase to a
  // DynamicBicycleCarState vector.
  return dynamic_cast<const DynamicBicycleCarState<T>&>(cstate.get_vector());
}

template <typename T>
DynamicBicycleCarState<T>& DynamicBicycleCar<T>::get_mutable_state(drake::systems::Context<T>* context) const {
  drake::systems::ContinuousState<T>* cstate = &context->get_mutable_continuous_state();
  return dynamic_cast<DynamicBicycleCarState<T>&>(cstate->get_mutable_vector());
}

template <typename T>
void DynamicBicycleCar<T>::CopyStateOut(const drake::systems::Context<T>& context,
                                        DynamicBicycleCarState<T>* output) const {
  output->SetFrom(get_state(context));
}

template <typename T>
T DynamicBicycleCar<T>::CalcTireSlip(const DynamicBicycleCarState<T>& state, const DynamicBicycleCarParams<T>& params,
                                     const T& steer_angle, Tire tire_select) {
  using std::atan2;

  if (tire_select == Tire::kFrontTire) {
    // Front tire slip angle.
    return atan2(state.v_LCp_y() + params.Lf() * state.yawDt_LC(), state.v_LCp_x()) - steer_angle;
  } else {
    // Rear tire slip angle.
    return atan2(state.v_LCp_y() - params.Lb() * state.yawDt_LC(), state.v_LCp_x());
  }
}

template <typename T>
T DynamicBicycleCar<T>::CalcNormalTireForce(const DynamicBicycleCarParams<T>& params, const T& f_Cp_x,
                                            Tire tire_select) {
  if (tire_select == Tire::kFrontTire) {
    // Front tire normal force.
    return (1 / (params.Lf() + params.Lb())) *
           (params.mass() * params.Lb() * params.gravity() - params.p_LoCp_z() * f_Cp_x);
  } else {
    // Rear tire normal force.
    return (1 / (params.Lf() + params.Lb())) *
           (params.mass() * params.Lf() * params.gravity() + params.p_LoCp_z() * f_Cp_x);
  }
}

template <typename T>
T DynamicBicycleCar<T>::CalcLateralTireForce(const T& tire_slip_angle, const T& c_alpha, const T& f_z, const T& mu) {
  // Based on Fiala non-linear brush tire model as presented by Pacejka [2].

  DRAKE_ASSERT(c_alpha >= 0.0);
  DRAKE_ASSERT(mu >= 0.0);
  DRAKE_ASSERT(f_z >= 0.0);  // non-negative normal force acting on the tire.

  using std::abs;
  using std::atan2;
  using std::pow;
  using std::tan;

  const T f_y_non_saturated_tire =
      -c_alpha * tan(tire_slip_angle) +
      ((c_alpha * c_alpha) / (3 * mu * f_z)) * abs(tan(tire_slip_angle)) * tan(tire_slip_angle) -
      (pow(c_alpha, 3) / (27 * (mu * mu) * (f_z * f_z))) * pow(tan(tire_slip_angle), 3);
  const int tire_slip_angle_sign = ((T(0) < tire_slip_angle) ? 1 : 0) - ((tire_slip_angle < T(0)) ? 1 : 0);
  const T f_y_saturated_tire = -mu * f_z * tire_slip_angle_sign;
  // Note: the cond function is used as an if-else statement in order to make
  // the conditional symbolic::Expression capable.
  return drake::cond(abs(tire_slip_angle) < atan2(3 * mu * f_z, c_alpha), f_y_non_saturated_tire, f_y_saturated_tire);
}

template <typename T>
void DynamicBicycleCar<T>::DoCalcTimeDerivatives(const drake::systems::Context<T>& context,
                                                 drake::systems::ContinuousState<T>* derivatives) const {
  using std::cos;
  using std::max;

  // Get the current state and derivative vectors of the system.
  const DynamicBicycleCarState<T>& state = get_state(context);
  DynamicBicycleCarState<T>& derivative_state =
      dynamic_cast<DynamicBicycleCarState<T>&>(derivatives->get_mutable_vector());

  // Obtain the car parameters.
  const DynamicBicycleCarParams<T>& params = this->template GetNumericParameter<DynamicBicycleCarParams>(context, 0);

  const T steer_CD = get_steer(context);
  const T f_Cp_x = get_longitudinal_force(context);

  // Calculate tire slip angles.
  const T tire_slip_angle_f = CalcTireSlip(state, params, steer_CD, Tire::kFrontTire);
  const T tire_slip_angle_r = CalcTireSlip(state, params, steer_CD, Tire::kRearTire);

  // Calculate tire forces.
  const T f_z_f = CalcNormalTireForce(params, f_Cp_x, Tire::kFrontTire);
  const T f_z_r = CalcNormalTireForce(params, f_Cp_x, Tire::kRearTire);
  const T f_y_f = CalcLateralTireForce(tire_slip_angle_f, params.c_alpha_f(), f_z_f, params.mu());
  const T f_y_r = CalcLateralTireForce(tire_slip_angle_r, params.c_alpha_r(), f_z_r, params.mu());

  // Catch to calculate sideslip angle when v_LoCp_x drops below 1 m/s.
  const T sideslip = state.v_LCp_y() / max(1.0, state.v_LCp_x());

  // Calculate state derivatives.
  derivative_state.set_p_LoCp_x(state.v_LCp_x());
  derivative_state.set_p_LoCp_y(state.v_LCp_y());
  derivative_state.set_yaw_LC(state.yawDt_LC());
  derivative_state.set_v_LCp_x((f_Cp_x / params.mass()) + state.yawDt_LC() * state.v_LCp_x() * sideslip);
  derivative_state.set_v_LCp_y((f_y_f * cos(steer_CD) + f_y_r) / params.mass() - state.yawDt_LC() * state.v_LCp_x());
  derivative_state.set_yawDt_LC((params.Lf() * f_y_f * cos(steer_CD) - params.Lb() * f_y_r) / params.izz());
}

}  // namespace delphyne

// Explicitly instantiate on default scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::delphyne::DynamicBicycleCar)
