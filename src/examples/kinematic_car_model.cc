// Copyright 2018 Toyota Research Institute
#include "examples/kinematic_car_model.h"

#include <algorithm>
#include <cmath>

#include "systems/calc_smooth_acceleration.h"

#include "drake/common/eigen_types.h"
#include "drake/math/saturate.h"

namespace delphyne {

void KinematicCarModel::set_driving_command(
    const ::backend::DrivingCommand& driving_command) {
  driving_command_.steering = drake::math::saturate(
      driving_command.steering, -properties_.max_abs_steering_angle,
      properties_.max_abs_steering_angle);
  driving_command_.acceleration = driving_command.acceleration;
}

void KinematicCarModel::StepBy(double time_step) {
  // Compute the smooth acceleration that the vehicle actually executes.
  const double desired_acceleration = driving_command_.acceleration;
  const double smooth_acceleration = drake::math::saturate(
      delphyne::calc_smooth_acceleration<double>(
          desired_acceleration, properties_.max_velocity,
          properties_.velocity_limit_kp, state_.velocity),
      -properties_.max_acceleration, properties_.max_acceleration);

  // Determine steering.
  const double curvature =
      std::tan(driving_command_.steering) / properties_.wheelbase;

  // Don't allow small negative velocities to affect position or heading.
  const double nonneg_velocity = std::max(0., state_.velocity);

  // Stepping the model state.
  const double yaw = state_.inertial_rotation.yaw_angle();
  drake::Vector3<double> step_position;
  step_position[0] = nonneg_velocity * std::cos(yaw) * time_step;
  step_position[1] = nonneg_velocity * std::sin(yaw) * time_step;
  // TODO(agalbachicar):    World is not flat so care about terrain changes.
  step_position[2] = 0.;
  state_.inertial_position += step_position;

  // TODO(agalbachicar):    World is not flat so care about terrain changes.
  const double step_heading = curvature * nonneg_velocity * time_step;
  state_.inertial_rotation.set(
      0. /* roll */, 0. /* pitch */, yaw + step_heading);

  state_.velocity += smooth_acceleration * time_step;
}

}  // namespace delphyne
