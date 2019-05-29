// Copyright 2018 Toyota Research Institute
#include "examples/opendriver_controller.h"

#include <utility>

#include "examples/pure_pursuit.h"
#include "opendrive/OdrCoord.hh"
#include "opendrive/OdrLaneCoord.hh"

namespace delphyne {

namespace {
// Returns an OpeDrive::Coord inertial pose from `inertial_position` and
// `inertial_rotation`.
OpenDrive::Coord PoseToOpenDRIVECoord(
    const drake::Vector3<double>& inertial_position,
    const drake::math::RollPitchYaw<double>& inertial_rotation) {
  return OpenDrive::Coord(inertial_position[0], inertial_position[1],
      inertial_position[2], inertial_rotation.yaw_angle(),
      inertial_rotation.pitch_angle(), inertial_rotation.roll_angle());
}

}  // namespace

backend::DrivingCommand OpenDriverController::Drive(
  const KinematicCarState& car_state) {
  // Compute the goal point with the strategy
  const OpenDrive::Coord xodr_inertial_coord = PoseToOpenDRIVECoord(
      car_state.inertial_position, car_state.inertial_rotation);

  state_.goal = PurePursuit::ComputeGoalPoint(
          parameters_.s_lookahead, xodr_inertial_coord, state_.goal.lane_coord,
          state_.goal.lane_direction, manager_, step_strategy_.get());

  manager_->setLanePos(state_.goal.lane_coord);
  DRAKE_DEMAND(manager_->lane2inertial());
  const OpenDrive::Coord inertial_target_goal = manager_->getInertialPos();

  const double steering = PurePursuit::Evaluate(
      parameters_.s_lookahead, parameters_.car_properties, xodr_inertial_coord,
      inertial_target_goal);
  const double acceleration = EvaluateAcceleration(
      xodr_inertial_coord, inertial_target_goal, state_.goal.type,
      car_state.velocity);

  return backend::DrivingCommand(steering, acceleration);
}

double OpenDriverController::EvaluateAcceleration(
    const OpenDrive::Coord& inertial_pose,
    const OpenDrive::Coord& target_goal,
    backend::GoalType goal_type,
    double speed) const {
  if (goal_type == backend::GoalType::kEndpoint) {  // Tries to stop the car.
    const double e_distance =
      (drake::Vector3<double>(inertial_pose.getX(), inertial_pose.getY(),
                              inertial_pose.getZ()) -
       drake::Vector3<double>(target_goal.getX(), target_goal.getY(),
                              target_goal.getZ())).norm();
    return -(speed * speed) / (2. * e_distance);
  }
  // Accelerates the car.
  return parameters_.default_acceleration;
}


}  // namespace delphyne
