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

#pragma once

#include <drake/systems/framework/leaf_system.h>

#include "gen/dynamic_bicycle_car_input.h"
#include "gen/dynamic_bicycle_car_params.h"
#include "gen/dynamic_bicycle_car_state.h"

namespace delphyne {

/// DynamicBicycleCar implements a planar rigid body bicycle model of an
/// automobile with a non-linear brush tire model from Bobier (2012) [1]. This
/// is a simplified model that assumes a vehicle that has two wheels: one at the
/// front, and one at the rear. Also, this three-DOF model captures the dynamics
/// in the lateral (Cy), longitudinal (Cx), and yaw (about Cz) directions but
/// not the roll (about Cx) and pitch (about Cy) directions.
///
/// There are three coordinate frames of interest in this model: a local frame L
/// fixed on earth with origin Lo, a frame attached to the vehicle's chassis C
/// with the origin of C being Co located at a distance of Lf from the front
/// axle along the center line of the vehicle, and a steering frame D with the
/// origin Do located at the front axle along the center line of the vehicle.
/// Note that the point Co is also referred to as the control point Cp, and
/// although the location of the vehicle's center of mass Ccm can move depending
/// on weight transfer dynamics, it is assumed that the location of Ccm is
/// coincident with Co and Ccp. L is a cartesian, right handed coordinate system
/// with Lz being gravity aligned (gravity acts in the negative Lz direction).
///
/// The states of the model are:
///   - Lx measure of the location of Cp from Lo `p_LoCp_x` [m]
///   - Ly measure of the location of Cp from Lo `p_LoCp_y` [m]
///   - Yaw angle from Lx to Cx with positive Lz sense `yaw_LC` [rad]
///   - Cx measure of Cp's velocity in L `v_LCp_x` [m/s]
///   - Cy measure of Cp's velocity in L `v_LCp_y` [m/s]
///   - C's angular velocity in frame L `yawDt_LC` [rad/s]
///
/// Inputs to this system:
///   - Steer angle from Cx to Dx with positive Cz sense `steer_CD` [rad]
///   - The Cx measure of the Longitudinal force on body C at Cp `f_Cp_x` [N]
///
/// Outputs of this system:
///   - A DynamicBicycleCarState containing the 6-dimensional state vector of
///     the vehicle.
///
/// Note that the vehicle's angular velocity in L `yawDt_LC` is sometimes
/// referred to as the yaw rate `r`, and the tire angle is sometimes referred
/// to as δ.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// [1] C. Bobier. A Phase Portrait Approach to Vehicle Stability
///     and Envelope Control. Ph. D. thesis (Stanford University), 2012.
///     pp. 22 - 25, pp. 35.
///
/// [2] H. Pacejka, Tire and vehicle dynamics, 3rd ed. Society of Automotive
///     Engineers and Butterworth-Heinemann, 2012.
///
/// [3] G. Heydinger, R. Bixel, W. Garrott, M. Pyne, J. Howe and D. Guenther,
///     "Measured Vehicle Inertial Parameters-NHTSA’s Data Through November
///     1998", SAE Technical Paper Series, 1999. p. 24.
///
/// @ingroup automotive_plants

template <typename T>
class DynamicBicycleCar final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DynamicBicycleCar);

  /// Default constructor.
  DynamicBicycleCar();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit DynamicBicycleCar(const DynamicBicycleCar<U>&) : DynamicBicycleCar<T>() {}

  /// Specifies whether to use the front or rear tire for calculating various
  /// parameters.
  enum class Tire {
    kFrontTire,
    kRearTire,
  };

  /// Returns the port to output the state.
  const drake::systems::OutputPort<T>& get_output_port() const;

  /// Returns the input port to the tire angle and applied longitudinal force.
  const drake::systems::InputPort<T>& get_input_port() const;

  const DynamicBicycleCarState<T>& get_state(const drake::systems::Context<T>& context) const;

  DynamicBicycleCarState<T>& get_mutable_state(drake::systems::Context<T>* context) const;

  /// Slip angle of front or rear tires.
  static T CalcTireSlip(const DynamicBicycleCarState<T>& state, const DynamicBicycleCarParams<T>& params,
                        const T& steer_angle, Tire tire_select);

  /// Normal forces on the front or rear tires.
  static T CalcNormalTireForce(const DynamicBicycleCarParams<T>& params, const T& f_x, Tire tire_select);

  /// Lateral tire forces on the front or rear tires.
  static T CalcLateralTireForce(const T& tire_slip_angle, const T& c_alpha, const T& f_z, const T& mu);

 private:
  // Evaluates the input port and returns the scalar value of the steering
  // angle.
  const T get_steer(const drake::systems::Context<T>& context) const {
    return this->EvalVectorInput(context, 0)->GetAtIndex(DynamicBicycleCarInputIndices::kSteerCd);
  }

  // Evaluates the input port and returns the scalar value of the longitudinal
  // force.
  const T get_longitudinal_force(const drake::systems::Context<T>& context) const {
    return this->EvalVectorInput(context, 0)->GetAtIndex(DynamicBicycleCarInputIndices::kFCpX);
  }

  // Copies the state out to the output port.
  void CopyStateOut(const drake::systems::Context<T>& context, DynamicBicycleCarState<T>* output) const;

  // Calculates the time derivatives of the state.
  void DoCalcTimeDerivatives(const drake::systems::Context<T>& context,
                             drake::systems::ContinuousState<T>* derivatives) const override;
};

}  // namespace delphyne
