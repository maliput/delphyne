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

#pragma once

#include <memory>

#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/rendering/frame_velocity.h>
#include <drake/systems/rendering/pose_vector.h>

#include "gen/angular_rate_acceleration_command.h"
#include "gen/simple_car_state.h"

namespace delphyne {

/// UnicylceCar is a parameter-free model of a car governed by the second-order equations
///
/// \f{eqnarray*}{
///   \dot{x} &=& v \cos(\theta) \\ %
///   \dot{y} &=& v \sin(\theta) \\ %
///   \dot{\theta} &=& u_{\omega} \\ %
///   \dot{v} &=& u_{a}
/// \f}
///
/// state vector (planar Cartesian):
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///
/// input vector:
/// * angular rotation rate: a positive angle means a positive change in
///   heading (left turn);
/// * acceleration
///
/// output port 0: same as state vector.
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///
/// @tparam T must support certain arithmetic operations;
/// for details, see drake::symbolic::Expression.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class UnicycleCar final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnicycleCar)

  /// @brief Initialise the background context on construction.
  ///
  /// The alternative is to back-fill these context values after the
  /// system has been added to a diagram and the subsystem context
  /// retrieved, but before execution.
  ///
  /// @param[in] initial_context_state The continuous state.
  UnicycleCar(const SimpleCarState<T>& initial_context_state = SimpleCarState<T>());

  // System<T> overrides
  void DoCalcTimeDerivatives(const drake::systems::Context<T>& context,
                             drake::systems::ContinuousState<T>* derivatives) const override;

  const drake::systems::OutputPort<T>& state_output() const;
  const drake::systems::OutputPort<T>& pose_output() const;
  const drake::systems::OutputPort<T>& velocity_output() const;

 private:
  void CalcStateOutput(const drake::systems::Context<T>&, SimpleCarState<T>*) const;
  void CalcPose(const drake::systems::Context<T>&, drake::systems::rendering::PoseVector<T>*) const;
  void CalcVelocity(const drake::systems::Context<T>&, drake::systems::rendering::FrameVelocity<T>*) const;

  void ImplCalcTimeDerivatives(const SimpleCarState<T>& state, const AngularRateAccelerationCommand<T>& input,
                               SimpleCarState<T>* rates) const;
};

}  // namespace delphyne
