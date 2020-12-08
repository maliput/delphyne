// Copyright 2020 Toyota Research Institute

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
  /// @param initial_context_state[in] The continuous state.
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
