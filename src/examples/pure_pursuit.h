// Copyright 2018 Toyota Research Institute
#pragma once

#include <utility>

#include "opendrive/OdrCoord.hh"
#include "opendrive/OdrLaneCoord.hh"
#include "opendrive/OdrManager.hh"

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "malidrive/backend/lane_direction.h"
#include "malidrive/backend/pure_pursuit_opendrive_goal.h"
#include "malidrive/backend/step_strategy.h"

#include "examples/kinematic_car_model.h"
#include "systems/lane_direction.h"

namespace delphyne {

/// PurePursuit computes the required steering angle to achieve a goal point on
/// an continuous planar path.  The path represents as the set of `t = 0`
/// positions along an OpenDRIVE lane, and a goal point is selected as a
/// pre-defined lookahead distance along the path in the intended direction of
/// travel. The algorithm outputs the steering angle required to guide the
/// vehicle toward the goal point based on its current position in global
/// coordinates.
///
/// See [1] and the corresponding .cc file for details on the algorithm.
///
/// They are already available to link against in the containing library.
///
/// [1] Coulter, R. Implementation of the Pure Pursuit Path Tracking
///     Algorithm. Carnegie Mellon University, Pittsburgh, Pennsylvania, Jan
///     1990.
///
/// Note: this class has been adapted from drake:automotive::PurePursuit to be
/// used with OpenDRIVE SDK.
class PurePursuit {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuit)

  PurePursuit() = delete;

  /// Evaluates the required steering angle in radians using the pure-pursuit
  /// method. Assumes zero elevation and superelevation.
  ///
  /// @param s_lookahead is the look ahead distance along the lane s-coordinate
  /// based on the closest position on the path to the vehicle. It must be
  /// positive.
  /// @param car_properties contains the `wheelbase` of the vehicle.
  /// @param inertial_pose is the inertial pose for the ego vehicle.
  /// @param track_id is the OpenDRIVE road track's ID. It must not be negative.
  /// @param lane_id is the OpenDRIVE road lane's ID.
  /// @param lane_direction is a LaneDirection containing the direction of
  /// travel along the positive-s coordinate.
  /// @param manager is the OpenDRIVE manager. It must not be nullptr.
  /// @return The steering angle command to exert on the vehicle.
  static double Evaluate(
    double s_lookahead,
    const KinematicCarModel::CarProperties& car_properties,
    const ::OpenDrive::Coord& inertial_pose,
    const ::OpenDrive::Coord& target_pose);

  /// Computes the goal point at a distance `s_lookahead` from the closest
  /// position on the curve in the intended direction of travel.
  ///
  /// Goal points are computed over the lane curve of the road. A goal point
  /// s-coordinate will be wrapped to the extents of the road when
  /// `step_strategy` does not offer any valid option to move in
  /// `lane_direction` direction.
  ///
  /// `inertial_coord` distance to `last_target_goal` might be greater than
  /// `s_lookahead`, so `last_target_goal` is returned. When it is equal or
  /// closer, a mapping of `inertial_coord` to `last_target_goal` road domain
  /// is tried. If it is successful, the algorithm steps `s_lookahead` distance
  /// with `step_strategy`. Otherwise, it steps a fraction of `s_lookahead`
  /// with `step_strategy` too.
  ///
  /// @param s_lookahead is the look ahead distance along the lane s-coordinate
  /// based on the closest position on the path to the vehicle. It must be
  /// positive.
  /// @param inertial_coord is the inertial pose for the ego vehicle.
  /// @param last_target_coord is the last goal's OpenDrive::LaneCoord.
  /// @param lane_direction is the last goal's LaneDirection containing the
  /// direction of travel along the positive-s coordinate.
  /// @param manager is the OpenDRIVE manager. It must not be nullptr.
  /// @return A backend::PurePursuitOpenDriveGoal with a valid
  /// OpenDrive::LaneCoord coordinate for the controller to follow.
  static backend::PurePursuitOpenDriveGoal ComputeGoalPoint(
      double s_lookahead, const ::OpenDrive::Coord& inertial_coord,
      const ::OpenDrive::LaneCoord& last_target_goal,
      backend::LaneDirection lane_direction,  ::OpenDrive::OdrManager* manager,
      ::backend::StepStrategy* step_strategy);
};

}  // namespace delphyne
