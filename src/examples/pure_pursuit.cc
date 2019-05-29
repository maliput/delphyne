// Copyright 2019 Toyota Research Institute
#include "examples/pure_pursuit.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "opendrive/BaseNodes/OdrLaneSection.hh"

#include "drake/math/saturate.h"

#include "malidrive/backend/xodr_inertial_to_lane_mapping.h"
#include "malidrive/backend/lane_direction.h"
#include "systems/lane_direction.h"

namespace delphyne {

double PurePursuit::Evaluate(
    double s_lookahead,
    const KinematicCarModel::CarProperties& car_properties,
    const OpenDrive::Coord& inertial_pose,
    const OpenDrive::Coord& target_pose) {
  DRAKE_DEMAND(s_lookahead > 0);

  const double delta_r =
      -(target_pose.getX() - inertial_pose.getX()) *
        std::sin(inertial_pose.getH()) +
      (target_pose.getY() - inertial_pose.getY()) *
        std::cos(inertial_pose.getH());

  const double curvature = 2. * delta_r / (s_lookahead * s_lookahead);

  // Return the steering angle.
  return std::atan(car_properties.wheelbase * curvature);
}

namespace {

// Steps s-coordinate of `lane_coord` `s_lookahead` distance in `lane_direction`
// direction.
// When the `step_strategy` offers no solution to a LaneSection end or Junction,
// the returned goal type would be backend::GoalType::kEndpoint and
// lane coordinate may be closer than `s_lookahead` distance from `lane_coord`.
std::pair<OpenDrive::LaneCoord, backend::GoalType> ForwardLaneCoord(
    double s_lookahead, const OpenDrive::LaneCoord& lane_coord,
    backend::LaneDirection lane_direction, OpenDrive::OdrManager* manager,
    backend::StepStrategy* step_strategy) {
  // TODO(agalbachicar)   Make sure which is the type of lane_coord (i.e.
  //                      backend::GoalType).
  backend::GoalType goal_type{backend::GoalType::kRoadpoint};
  double s_new = lane_coord.getS();
  double pending_s_step = s_lookahead;
  // Sets the target over the lane.
  OpenDrive::LaneCoord target_lane_coord(
      lane_coord.getTrackId(), lane_coord.getLaneId(),
      lane_coord.getS());
  manager->setLanePos(target_lane_coord);
  DRAKE_DEMAND(manager->lane2inertial());

  while (pending_s_step > 0.) {
    bool reached_end_of_lane_section{false};
    if (lane_direction == backend::LaneDirection::kWithS) {
      if ((s_new + pending_s_step) > manager->getLaneSection()->mSEnd) {
        pending_s_step =
            pending_s_step + s_new - manager->getLaneSection()->mSEnd;
        s_new = manager->getLaneSection()->mSEnd;
        reached_end_of_lane_section = true;
      } else {
        s_new += pending_s_step;
        pending_s_step = 0.;
      }
    } else {
      if ((s_new - pending_s_step) < manager->getLaneSection()->mS) {
        pending_s_step =
            pending_s_step - s_new + manager->getLaneSection()->mS;
        s_new = manager->getLaneSection()->mS;
        reached_end_of_lane_section = true;
      } else {
        s_new -= pending_s_step;
        pending_s_step = 0.;
      }
    }

    // Move the manger position to the new lane position.
    target_lane_coord.setS(s_new);
    DRAKE_DEMAND(manager->lane2inertial());

    if (reached_end_of_lane_section) {
      drake::optional<OpenDrive::LaneCoord> lane_coord =
        step_strategy->SolveEndOfLane(target_lane_coord, lane_direction);
      if (lane_coord.has_value()) {
        // Advance the pose of the agent to the ongoing lane.
        s_new = lane_coord->getS();
        target_lane_coord = *lane_coord;
        // Move the manger position to the new lane position.
        manager->setLanePos(target_lane_coord);
        DRAKE_DEMAND(manager->lane2inertial());
      } else {
        // There is nowhere to go, so we just get out
        goal_type = backend::GoalType::kEndpoint;
        break;
      }
    }
  }
  return {target_lane_coord, goal_type};
}

}  // namespace


backend::PurePursuitOpenDriveGoal PurePursuit::ComputeGoalPoint(
    double s_lookahead, const OpenDrive::Coord& inertial_coord,
    const OpenDrive::LaneCoord& last_target_goal,
    backend::LaneDirection lane_direction,  OpenDrive::OdrManager* manager,
    backend::StepStrategy* step_strategy) {
  DRAKE_DEMAND(s_lookahead > 0);
  DRAKE_DEMAND(manager != nullptr);
  DRAKE_DEMAND(step_strategy != nullptr);

  // Computes the distance from the car to the last goal.
  manager->setLanePos(last_target_goal);
  DRAKE_DEMAND(manager->lane2inertial());
  const OpenDrive::Coord last_inertial_target_goal = manager->getInertialPos();
  const double car_to_last_goal_distance = OpenDrive::Coord::getDist(
      inertial_coord, last_inertial_target_goal);

  if (car_to_last_goal_distance < s_lookahead) {
    // Given that we are within the s_lookahead range, we need to find another
    // goal.
    const drake::optional<OpenDrive::LaneCoord> car_lane_coord =
        backend::XODRInertialToLaneCoordinate(
            inertial_coord, last_target_goal.getTrackId(),
            last_target_goal.getLaneId(), manager);
    // We try to map the car on the road. If we could, we should move forward
    // the target over the path. Otherwise, we just move the lookahead a bit
    // ahead (a fraction of the lookahead distance) of the previous target.
    const std::pair<OpenDrive::LaneCoord, backend::GoalType> goal =
        car_lane_coord.has_value() ?
            ForwardLaneCoord(s_lookahead, *car_lane_coord, lane_direction,
                             manager, step_strategy) :
            // TODO(agalbachicar)   Lookahead distance needs a better solution
            //                      based on neighbor LANE-Frame mappings. As
            //                      for now, a tenth is left here for
            //                      exploration purposes.
            ForwardLaneCoord(s_lookahead / 10., last_target_goal,
                             lane_direction, manager, step_strategy);
    return backend::PurePursuitOpenDriveGoal(goal.first, lane_direction,
                                             goal.second);
  }
  // We still are far away to care about mapping.
  return backend::PurePursuitOpenDriveGoal(
      last_target_goal, lane_direction, backend::GoalType::kRoadpoint);
}

}  // namespace delphyne
