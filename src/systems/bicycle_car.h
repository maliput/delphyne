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

#include <memory>

#include <Eigen/Geometry>
#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>

#include "gen/bicycle_car_parameters.h"
#include "gen/bicycle_car_state.h"

namespace delphyne {

/// BicycleCar implements a nonlinear rigid body bicycle model from Althoff &
/// Dolan (2014) [1].  The three-DOF model captures the rigid-body dynamics in
/// the lateral, longitudinal, and yaw directions but not in the roll and pitch
/// directions.  The model assumes a vehicle that has two wheels: one at the
/// front and one at the rear.  It has been demonstrated (e.g. [2]) that the
/// representation reasonably approximates the dynamics of a four-wheeled
/// vehicle; hence the model is useful as a simplified abstraction of car
/// dynamics.
///
/// The states of the model are:
///  - yaw angle Ψ [rad]
///  - yaw rate Ψ_dot [rad/s]
///  - slip angle at the center of mass β [rad]
///  - velocity magnitude (vector magnitude at the slip angle) vel [m/s]
///  - x-position of the center of mass sx [m]
///  - y-position of the center of mass sy [m]
///
/// N.B. "slip angle" (β) is the angle made between the body and the velocity
/// vector.  Thus, the velocity vector can be resolved into the body-relative
/// components `vx_body = cos(β)` and `vy_body = sin(β)`.  `β = 0` means the
/// velocity vector is pointing along the bicycle's longitudinal axis.
///
/// Inputs:
///  - Angle of the front wheel of the bicycle δ [rad]
///    (InputPort getter: get_steering_input_port())
///  - Force acting on the rigid body F_in [N]
///    (InputPort getter: get_force_input_port())
///
/// Output:
///  - A BicycleCarState containing the 6-dimensional state vector of the
///    bicycle.
///    (OutputPort getter: get_state_output_port())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// [1] M. Althoff and J.M. Dolan, Online verification of automated road
///     vehicles using reachability analysis, IEEE Transactions on Robotics,
///     30(4), 2014, pp. 903-908.  DOI: 10.1109/TRO.2014.2312453.
///
/// [2] M. Althoff and J. M. Dolan, Reachability computation of low-order
///     models for the safety verification of high-order road vehicle models,
///     in Proc. of the American Control Conference, 2012, pp. 3559–3566.
///
/// @ingroup automotive_plants
template <typename T>
class BicycleCar final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BicycleCar)

  /// Default constructor.
  BicycleCar();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit BicycleCar(const BicycleCar<U>&);

  ~BicycleCar() override;

  /// Returns the input port that contains the steering angle.
  const drake::systems::InputPort<T>& get_steering_input_port() const;

  /// Returns the input port that contains the applied powertrain force.
  const drake::systems::InputPort<T>& get_force_input_port() const;

  /// Returns the output port that contains the bicycle states.
  const drake::systems::OutputPort<T>& get_state_output_port() const;

 private:
  void CopyOutState(const drake::systems::Context<T>& context, BicycleCarState<T>* output) const;

  void DoCalcTimeDerivatives(const drake::systems::Context<T>& context,
                             drake::systems::ContinuousState<T>* derivatives) const override;

  void ImplCalcTimeDerivatives(const BicycleCarParameters<T>& params, const BicycleCarState<T>& state,
                               const drake::systems::BasicVector<T>& steering,
                               const drake::systems::BasicVector<T>& force, BicycleCarState<T>* derivatives) const;

  int steering_input_port_{};
  int force_input_port_{};
  int state_output_port_{};
};

}  // namespace delphyne
