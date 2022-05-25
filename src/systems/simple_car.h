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

#pragma once

#include <memory>

#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/rendering/frame_velocity.h>
#include <drake/systems/rendering/pose_vector.h>

#include "gen/driving_command.h"
#include "gen/simple_car_params.h"
#include "gen/simple_car_state.h"

namespace delphyne {

/// SimpleCar models an idealized response to driving commands, neglecting all
/// physics. Note that SimpleCar can move forward, stop, turn left, and turn
/// right but *cannot* travel in reverse.
///
/// parameters:
/// * uses systems::Parameters wrapping a SimpleCarParams
///
/// state vector (planar for now):
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
//    heading is defined around the +z axis, so positive-turn-left
/// * velocity
///
/// input vector:
/// * steering angle (virtual center wheel angle);
///   a positive angle means a positive change in heading (left turn);
///   the value must lie within (-pi, +pi).
/// * throttle (0-1)
/// * brake (0-1)
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
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class SimpleCar2 final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleCar2)

  /// @brief Initialise the background context on construction.
  ///
  /// The alternative is to back-fill these context values after the
  /// system has been added to a diagram and the subsystem context
  /// retrieved, but before execution.
  ///
  /// @param[in] initial_context_state The continuous state.
  /// @param[in] initial_context_parameters The numeric parameterization.
  SimpleCar2(const SimpleCarState<T>& initial_context_state = SimpleCarState<T>(),
             const SimpleCarParams<T>& initial_context_parameters = SimpleCarParams<T>());

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit SimpleCar2(const SimpleCar2<U>&);

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

  void ImplCalcTimeDerivatives(const SimpleCarParams<T>& params, const SimpleCarState<T>& state,
                               const DrivingCommand<T>& input, SimpleCarState<T>* rates) const;

  void CalcSteeringAngleConstraint(const drake::systems::Context<T>&, drake::VectorX<T>*) const;
  void CalcAccelerationConstraint(const drake::systems::Context<T>&, drake::VectorX<T>*) const;
  void CalcVelocityConstraint(const drake::systems::Context<T>&, drake::VectorX<T>*) const;
};

}  // namespace delphyne
