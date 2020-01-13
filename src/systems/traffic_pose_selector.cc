// Copyright 2018 Toyota Research Institute

#include "systems/traffic_pose_selector.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <drake/common/autodiffxd_make_coherent.h>
#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>
#include <drake/common/extract_double.h>
#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/segment.h>

namespace delphyne {

using drake::Quaternion;
using drake::Vector3;
using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseBundle;
using drake::systems::rendering::PoseVector;
using maliput::api::GeoPosition;
using maliput::api::GeoPositionT;
using maliput::api::HBounds;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::LanePosition;
using maliput::api::LanePositionResultT;
using maliput::api::LanePositionT;
using maliput::api::RBounds;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using maliput::api::Rotation;

using delphyne::AheadOrBehind;
using delphyne::ClosestPose;
using delphyne::RoadOdometry;

namespace {

// Returns `true` if and only if @p lane_position is within the longitudinal
// (s), lateral segment (r) and elevation (h) bounds of the specified @p lane
// (i.e. within @p linear_tolerance of `lane->segment_bounds()` and
// `lane->elevation_bounds()`).
template <typename T>
bool IsWithinSegmentBounds(const Lane* lane, const LanePositionT<T>& lane_position, const double linear_tolerance) {
  const double s = drake::ExtractDoubleOrThrow(lane_position.s());
  if (s < -linear_tolerance || s > lane->length() + linear_tolerance) {
    return false;
  }
  const double r = drake::ExtractDoubleOrThrow(lane_position.r());
  const RBounds r_bounds = lane->segment_bounds(s);
  if (r < r_bounds.min() - linear_tolerance || r > r_bounds.max() + linear_tolerance) {
    return false;
  }
  const HBounds h_bounds = lane->elevation_bounds(s, r);
  const double h = drake::ExtractDoubleOrThrow(lane_position.h());
  return (h >= h_bounds.min() - linear_tolerance && h <= h_bounds.max() + linear_tolerance);
}

// Returns `true` if and only if @p geo_position is within the longitudinal (s),
// lateral (r) and elevation (h) bounds of the specified @p lane (i.e. within
// @p linear_tolerance of `lane->lane_bounds()` and `lane->elevation_bounds()`).
// Optionally (i.e. if not nullptr), the corresponding @p nearest_lane_position
// is returned.
template <typename T>
bool IsWithinLaneBounds(const Lane* lane, const GeoPositionT<T>& geo_position, double linear_tolerance,
                        LanePositionT<T>* nearest_lane_position) {
  LanePositionResultT<T> result = lane->ToLanePositionT<T>(geo_position);
  if (result.distance < linear_tolerance) {
    const RBounds r_bounds = lane->lane_bounds(drake::ExtractDoubleOrThrow(result.lane_position.s()));
    if (result.lane_position.r() >= r_bounds.min() - linear_tolerance &&
        result.lane_position.r() <= r_bounds.max() + linear_tolerance) {
      if (nearest_lane_position != nullptr) {
        *nearest_lane_position = result.lane_position;
      }
      return true;
    }
  }
  return false;
}

// Returns `true` if and only if @p lane_position is within @p linear_tolerance
// of the lateral segment bounds of @p lane and, in addition, `r` is within its
// lane bounds.
template <typename T>
bool IsWithinLaneBounds(const Lane* lane, const LanePositionT<T>& lane_position, double linear_tolerance) {
  if (IsWithinSegmentBounds(lane, lane_position, linear_tolerance)) {
    const RBounds r_bounds = lane->lane_bounds(drake::ExtractDoubleOrThrow(lane_position.s()));
    return (lane_position.r() >= r_bounds.min() - linear_tolerance ||
            lane_position.r() <= r_bounds.max() + linear_tolerance);
  }
  return false;
}

// Returns `true` if and only if @p lane_position is within its road geometry
// linear_tolerance() of the lateral segment bounds of @p lane and, in addition,
// `r` is within its lane bounds.
template <typename T>
bool IsWithinLaneBounds(const Lane* lane, const LanePositionT<T>& lane_position) {
  const double linear_tolerance = lane->segment()->junction()->road_geometry()->linear_tolerance();
  return IsWithinLaneBounds(lane, lane_position, linear_tolerance);
}

// Given a @p lane_end, returns the LaneEnd corresponding to the
// exit-point of an ongoing lane. If the LaneEnd corresponds to a default
// branch at that end, then it is returned. If there is no default branch, the
// ongoing LaneEnd with index = 0 is selected. Otherwise, returns a LaneEnd
// with its `lane` member-field set to `nullptr`.
LaneEnd GetDefaultOrFirstOngoingLaneEndAhead(const LaneEnd& lane_end) {
  DRAKE_DEMAND(lane_end.lane != nullptr);
  std::optional<LaneEnd> branch = lane_end.lane->GetDefaultBranch(lane_end.end);
  if (!branch) {
    const LaneEndSet* branches = lane_end.lane->GetOngoingBranches(lane_end.end);
    if (branches->size() == 0) {
      return {nullptr, LaneEnd::kStart};
    }
    branch = branches->get(0);
  }
  // The LaneEnd of the found successor lane corresponds to the traversal end;
  // need to reverse this to get the opposite end (i.e. the one connected to the
  // branch point).
  branch->end = (branch->end == LaneEnd::kStart) ? LaneEnd::kFinish : LaneEnd::kStart;
  return *branch;
}

// Finds and returns the LaneEnd the vehicle is heading towards, based on the
// vehicle's current @p lane, @p lane_position and @p rotation (in global
// coordinates), and the @p side of the car (ahead or behind) that traffic is
// being observed.
template <typename T>
LaneEnd FindLaneEnd(const Lane* lane, const LanePositionT<T>& lane_position, const Quaternion<T>& rotation,
                    AheadOrBehind side) {
  // Get the vehicle's heading with respect to the current lane; use it to
  // determine if the vehicle is facing towards or against the lane's
  // canonical direction.
  const Quaternion<double> lane_rotation = lane->GetOrientation(lane_position.MakeDouble()).quat();
  // It is assumed that the vehicle is going in the lane's direction if
  // the angular distance θ between their headings is -π/2 ≤ θ ≤ π/2.
  const double angle = std::fabs(lane_rotation.angularDistance(
      Quaternion<double>(drake::ExtractDoubleOrThrow(rotation.w()), drake::ExtractDoubleOrThrow(rotation.x()),
                         drake::ExtractDoubleOrThrow(rotation.y()), drake::ExtractDoubleOrThrow(rotation.z()))));
  // True if one or the other, but not both.
  const bool positive_s_direction = (side == AheadOrBehind::kAhead) ^ (angle > M_PI / 2.);
  return {lane, (positive_s_direction) ? LaneEnd::kFinish : LaneEnd::kStart};
}

// Returns a RoadOdometry that contains an infinite `s` position, zero `r` and
// `h` positions, and zero velocities. If @p lane_end_ahead refers to
// to an end (i.e. LaneEnd::kFinish), a RoadOdometry containing an s-position
// at positive infinity is returned; otherwise a negative-infinite position is
// returned. For T == AutoDiffXd, the derivatives of the returned RoadOdometry
// are made to be coherent with respect to @p pose.
template <typename T>
RoadOdometry<T> MakeInfiniteOdometry(const LaneEnd& lane_end_ahead, const PoseVector<T>& pose) {
  const T witness_state = pose.get_isometry().translation().x();

  T zero_position(0.);
  drake::autodiffxd_make_coherent(witness_state, &zero_position);
  T infinite_position = std::numeric_limits<T>::infinity();
  if (lane_end_ahead.end == LaneEnd::kStart) {
    infinite_position = -infinite_position;
  }
  drake::autodiffxd_make_coherent(witness_state, &infinite_position);
  const LanePositionT<T> lane_position(infinite_position, zero_position, zero_position);
  FrameVelocity<T> frame_velocity;
  auto velocity = frame_velocity.get_mutable_value();
  for (int i = 0; i < frame_velocity.kSize; ++i) {
    drake::autodiffxd_make_coherent(witness_state, &velocity(i));
  }
  // TODO(jadecastro) Consider moving the above autodiffxd_make_coherent() step
  // to BasicVector().
  return {lane_end_ahead.lane, lane_position, frame_velocity};
}

// Returns positive infinity. For T = AutoDiffXd, the derivatives of the the
// return value are made to be coherent with respect to @p pose.
template <typename T>
T MakeInfiniteDistance(const PoseVector<T>& pose) {
  T infinite_distance = std::numeric_limits<T>::infinity();
  drake::autodiffxd_make_coherent(pose.get_isometry().translation().x(), &infinite_distance);
  return infinite_distance;
}

// Returns the distance (along the `s`-coordinate) from an end of a lane to a @p
// lane_position in that lane, where the end is determined by @p lane_end_ahead.
// @pre Given @p lane_position is within `lane_end_ahead.lane` lateral segment
//      bounds.
template <typename T>
T CalcLaneProgress(const LaneEnd& lane_end_ahead, const LanePositionT<T>& lane_position) {
  if (lane_end_ahead.end == LaneEnd::kStart) {
    return T(lane_end_ahead.lane->length()) - lane_position.s();
  }
  return lane_position.s();
}

// Returns true if `lane0` has an equal identifier as `lane1`, and false
// otherwise. The result is trivially false if either is nullptr.
bool IsEqual(const Lane* lane0, const Lane* lane1) {
  if (!lane0 || !lane1) return false;
  return lane0->id() == lane1->id();
}

// Returns the closest pose to the ego car along the default path given a
// `lane`, the ego vehicle's pose `ego_pose`, a PoseBundle of `traffic_poses`,
// the AheadOrBehind specifier `side`. The return value is the same as
// TrafficPoseSelector<T>::FindSingleClosestPose().
template <typename T>
ClosestPose<T> FindSingleClosestInDefaultPath(const Lane* ego_lane, const PoseVector<T>& ego_pose,
                                              const PoseBundle<T>& traffic_poses, const T& scan_distance,
                                              const AheadOrBehind side) {
  DRAKE_DEMAND(ego_lane != nullptr);
  const double linear_tolerance = ego_lane->segment()->junction()->road_geometry()->linear_tolerance();

  const GeoPositionT<T> ego_geo_position = GeoPositionT<T>::FromXyz(ego_pose.get_isometry().translation());
  const LanePositionT<T> ego_lane_position = ego_lane->ToLanePositionT<T>(ego_geo_position).lane_position;

  std::vector<GeoPositionT<T>> traffic_geo_positions;
  traffic_geo_positions.reserve(traffic_poses.get_num_poses());
  for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
    const drake::Isometry3<T>& traffic_isometry = traffic_poses.get_pose(i);
    traffic_geo_positions.push_back(GeoPositionT<T>::FromXyz(traffic_isometry.translation()));
  }

  const LaneEnd ego_lane_end_ahead = FindLaneEnd(ego_lane, ego_lane_position, ego_pose.get_rotation(), side);
  const T ego_lane_progress = CalcLaneProgress(ego_lane_end_ahead, ego_lane_position);

  ClosestPose<T> result;
  result.odometry = MakeInfiniteOdometry(ego_lane_end_ahead, ego_pose);
  result.distance = MakeInfiniteDistance(ego_pose);
  // N.B. ego_s is negated to recover the remaining distance to the end of the
  // lane  when `distance_scanned` is incremented by the ego car's lane length.
  T distance_scanned = T(-ego_lane_progress);

  // Traverse forward or backward from the current lane the given scan_distance,
  // looking for traffic cars.
  LaneEnd next_lane_end_ahead = ego_lane_end_ahead;
  while (next_lane_end_ahead.lane && distance_scanned < scan_distance) {
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      const GeoPositionT<T>& traffic_geo_position = traffic_geo_positions[i];

      // Skip the ego car itself (also found in the traffic cars' array).
      if (traffic_geo_position == ego_geo_position) continue;

      // Skip traffic cars that are not in the current lane.
      LanePositionT<T> traffic_lane_position;
      if (!IsWithinLaneBounds(next_lane_end_ahead.lane, traffic_geo_position, linear_tolerance,
                              &traffic_lane_position)) {
        continue;
      }

      const T traffic_lane_progress = CalcLaneProgress(next_lane_end_ahead, traffic_lane_position);
      T traffic_distance = traffic_lane_progress + distance_scanned;
      if (distance_scanned <= 0.) {
        const T traffic_lane_progress_delta = traffic_lane_progress - ego_lane_progress;
        // Ignore traffic cars that are not in the desired direction (ahead or
        // behind) of the ego car (with respect to the car's current direction).
        // Cars with identical s-values as the ego but shifted laterally are
        // treated as `kBehind` cars. Note that this check is only needed when
        // the two share the same lane or, equivalently, `distance_scanned <= 0`
        if (traffic_lane_progress_delta < 0. || (side == AheadOrBehind::kAhead && traffic_lane_progress_delta == 0.)) {
          continue;
        }
        traffic_distance = traffic_lane_progress_delta;
      }

      // Ignore positions at the desired direction (ahead or behind) of the ego
      // car that are not closer than any other found so far.
      if (traffic_distance < scan_distance && result.distance > traffic_distance) {
        // Update the result and incremental distance with the new candidate.
        result.odometry =
            RoadOdometry<T>(next_lane_end_ahead.lane, traffic_lane_position, traffic_poses.get_velocity(i));
        result.distance = traffic_distance;
      }
    }

    // Figure out whether or not the result is within scan_distance.
    if (result.distance < scan_distance) break;

    // Increment distance_scanned.
    distance_scanned += T(next_lane_end_ahead.lane->length());

    // Obtain the next lane end ahead in the scanned sequence.
    next_lane_end_ahead = GetDefaultOrFirstOngoingLaneEndAhead(next_lane_end_ahead);
  }
  return result;
}

// Extracts the vehicle's `s`-direction velocity based on its current @p lane,
// @p lane_position and @p velocity.
// @pre Given @p lane is not nullptr.
// @pre Given @p lane_position is within @p lane bounds.
// @pre Road has zero elevation and superelevation.
template <typename T>
T CalcSigmaVelocity(const Lane* lane, const LanePositionT<T>& lane_position, const FrameVelocity<T>& velocity) {
  DRAKE_DEMAND(lane != nullptr);
  const Vector3<T> linear_velocity = velocity.get_velocity().translational();
  const Rotation rotation = lane->GetOrientation(lane_position.MakeDouble());
  return (linear_velocity(0) * std::cos(rotation.yaw()) + linear_velocity(1) * std::sin(rotation.yaw()));
  // TODO(jadecastro) Replace above with the dot product of vel dotted with the
  // unit vector of the s coordinate, i.e.
  // (cos β * cos γ, cos β * sin γ, -sin β), where β is pitch and γ is yaw.
}

// Assumed ego velocity, used in determining how far ahead to search for traffic
// cars.
static constexpr double kEgoSigmaVelocity{1.};

// A container consisting of a maliput::api::LaneEnd and a distance along the
// s-coordinate to that end.
template <typename T>
using LaneEndDistance = std::pair<const T, const maliput::api::LaneEnd>;

// Returns the closest pose to the ego car given a `lane`, the ego vehicle's
// pose `ego_pose`, a PoseBundle of `traffic_poses`, the AheadOrBehind specifier
// `side`, and a set of `branches` to be checked. The return value is the same
// as TrafficPoseSelector<T>::FindSingleClosestPose().
template <typename T>
ClosestPose<T> FindSingleClosestInBranches(const Lane* ego_lane, const PoseVector<T>& ego_pose,
                                           const PoseBundle<T>& traffic_poses,
                                           const std::vector<LaneEndDistance<T>>& distance_to_branches,
                                           const T& scan_distance, const AheadOrBehind side) {
  DRAKE_DEMAND(ego_lane != nullptr);
  DRAKE_DEMAND(!distance_to_branches.empty());
  using std::abs;
  using std::min;

  const GeoPositionT<T> ego_geo_position = GeoPositionT<T>::FromXyz(ego_pose.get_isometry().translation());
  const LanePositionT<T> ego_lane_position = ego_lane->ToLanePositionT<T>(ego_geo_position).lane_position;
  const LaneEnd ego_lane_end_ahead = FindLaneEnd(ego_lane, ego_lane_position, ego_pose.get_rotation(), side);

  ClosestPose<T> result;
  result.odometry = MakeInfiniteOdometry(ego_lane_end_ahead, ego_pose);
  result.distance = MakeInfiniteDistance(ego_pose);

  const RoadGeometry* road_geometry = ego_lane->segment()->junction()->road_geometry();

  for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
    const drake::Isometry3<T>& traffic_isometry = traffic_poses.get_pose(i);
    const GeoPositionT<T> traffic_geo_position = GeoPositionT<T>::FromXyz(traffic_isometry.translation());
    const RoadPosition traffic_road_position =
        road_geometry->ToRoadPosition(traffic_geo_position.MakeDouble()).road_position;
    const Lane* traffic_lane = traffic_road_position.lane;
    // TODO(jadecastro) Supply a valid hint.
    if (!traffic_lane) continue;

    // TODO(jadecastro) RoadGeometry::ToRoadPositionT() doesn't yet exist, so
    // for now, just call Lane::ToLanePositionT.
    const LanePositionT<T> traffic_lane_position = traffic_lane->ToLanePositionT<T>(traffic_geo_position).lane_position;

    // Get this traffic vehicle's velocity and travel direction in the lane it
    // is occupying.
    const T traffic_lane_sigma_v =
        CalcSigmaVelocity(traffic_lane, traffic_lane_position, traffic_poses.get_velocity(i));

    LaneEnd traffic_lane_end_ahead = FindLaneEnd(traffic_lane, traffic_lane_position,
                                                 Quaternion<T>(traffic_isometry.linear()), AheadOrBehind::kAhead);
    const T traffic_lane_progress = CalcLaneProgress(traffic_lane_end_ahead, traffic_lane_position);

    // Determine if any of the traffic cars eventually lead to a branch within a
    // speed- and branch-dependent influence distance horizon.
    for (const LaneEndDistance<T>& distance_to_branch : distance_to_branches) {
      T distance_scanned = -traffic_lane_progress;

      LaneEnd branch;
      T ego_distance_to_branch{};
      std::tie(ego_distance_to_branch, branch) = distance_to_branch;
      // The distance ahead needed to scan for intersection is assumed equal to
      // the distance scanned in the ego vehicle's lane times the ratio of
      // s-velocity of the traffic car to that of the ego car. Cars much slower
      // than the ego car are thus phased out closer to the branch-point, while
      // those that are faster remain in scope further away from the
      // branch-point.
      //
      // TODO(jadecastro) Use the actual velocity from the ego car, ensuring
      // that distance_to_scan is negative if the ego is moving away from the
      // branch point.
      const T distance_to_scan =
          min(scan_distance, abs(traffic_lane_sigma_v / T(kEgoSigmaVelocity)) * ego_distance_to_branch);

      T effective_headway = MakeInfiniteDistance(ego_pose);
      LaneEnd next_traffic_lane_end_ahead = traffic_lane_end_ahead;
      while (next_traffic_lane_end_ahead.lane && distance_scanned < distance_to_scan) {
        const Lane* trial_lane = next_traffic_lane_end_ahead.lane;
        const LaneEnd::Which trial_lane_end = next_traffic_lane_end_ahead.end;
        // If this vehicle is in the trial_lane, then use it to compute the
        // effective headway distance to the ego vehicle. Otherwise continue
        // down its path looking for the lane connected to a branch up to
        // distance_to_scan.
        if (IsEqual(trial_lane, branch.lane) && (trial_lane_end == branch.end)) {
          const T distance_to_lane_end = distance_scanned + T(trial_lane->length());
          // "Effective headway" is the distance between the traffic vehicle and
          // the ego vehicle, compared relative to their positions with respect
          // to their shared branch point.
          effective_headway = ego_distance_to_branch - distance_to_lane_end;
        }
        if (0. < effective_headway && effective_headway < result.distance) {
          result.distance = effective_headway;
          result.odometry = RoadOdometry<T>(traffic_lane, traffic_lane_position, traffic_poses.get_velocity(i));
          break;
        }
        // Increment distance_scanned.
        distance_scanned += T(trial_lane->length());

        next_traffic_lane_end_ahead = GetDefaultOrFirstOngoingLaneEndAhead(next_traffic_lane_end_ahead);
      }
    }
  }
  return result;
}

// Returns the vector of branches along the sequence of default road segments in
// a `road`, up to a given `scan_distance` in the ego vehicle's current lane,
// given its PoseVector `ego_pose` and AheadOrBehind `side`. A vector of
// LaneEndDistance is returned, whose elements are pairs where the first entry
// is the distance along the s-coordinate from the ego vehicle to the branch and
// second entry is the LaneEnd describing the branch.
template <typename T>
std::vector<LaneEndDistance<T>> FindConfluentBranches(const Lane* ego_lane, const PoseVector<T>& ego_pose,
                                                      const T& scan_distance, const AheadOrBehind side) {
  DRAKE_DEMAND(ego_lane != nullptr);  // The ego car must be in a lane.
  const GeoPositionT<T> ego_geo_position = GeoPositionT<T>::FromXyz(ego_pose.get_isometry().translation());
  const LanePositionT<T> ego_lane_position = ego_lane->ToLanePositionT<T>(ego_geo_position).lane_position;
  const LaneEnd ego_lane_end_ahead = FindLaneEnd(ego_lane, ego_lane_position, ego_pose.get_rotation(), side);
  const T ego_lane_progress = CalcLaneProgress(ego_lane_end_ahead, ego_lane_position);

  // N.B. ego_s is negated to recover the remaining distance to the end of the
  // lane  when `distance_scanned` is incremented by the ego car's lane length.
  T distance_scanned = T(-ego_lane_progress);
  // Obtain any branches starting from the ego vehicle's lane, moving along its
  // direction of travel by an amount equal to scan_distance.
  std::vector<LaneEndDistance<T>> branches;
  LaneEnd next_lane_end_ahead = ego_lane_end_ahead;
  while (next_lane_end_ahead.lane && distance_scanned < scan_distance) {
    // Increment distance_scanned and collect all non-trivial branches as we go.
    distance_scanned += T(next_lane_end_ahead.lane->length());
    const LaneEndSet* ends = next_lane_end_ahead.lane->GetConfluentBranches(next_lane_end_ahead.end);
    if (ends != nullptr && ends->size() > 1) {
      for (int i = 0; i < ends->size(); ++i) {
        // Store, from the complete list, the LaneEnds that do not belong to the
        // main path (lane sequence containing the ego vehicle).
        if (!IsEqual(next_lane_end_ahead.lane, ends->get(i).lane)) {
          branches.emplace_back(std::make_pair(distance_scanned, ends->get(i)));
        }
      }
    }
    next_lane_end_ahead = GetDefaultOrFirstOngoingLaneEndAhead(next_lane_end_ahead);
  }
  return branches;
}

}  // namespace

template <typename T>
std::map<AheadOrBehind, const ClosestPose<T>> TrafficPoseSelector<T>::FindClosestPair(
    const Lane* lane, const PoseVector<T>& ego_pose, const PoseBundle<T>& traffic_poses, const T& scan_distance,
    ScanStrategy path_or_branches) {
  return {{AheadOrBehind::kAhead, FindSingleClosestPose(lane, ego_pose, traffic_poses, scan_distance,
                                                        AheadOrBehind::kAhead, path_or_branches)},
          {AheadOrBehind::kBehind, FindSingleClosestPose(lane, ego_pose, traffic_poses, scan_distance,
                                                         AheadOrBehind::kBehind, path_or_branches)}};
}

template <typename T>
ClosestPose<T> TrafficPoseSelector<T>::FindSingleClosestPose(const Lane* lane, const PoseVector<T>& ego_pose,
                                                             const PoseBundle<T>& traffic_poses, const T& scan_distance,
                                                             const AheadOrBehind side, ScanStrategy path_or_branches) {
  DRAKE_THROW_UNLESS(lane != nullptr);  // The ego car must be in a lane.

  // Find any leading traffic cars along the same default path as the ego
  // vehicle.
  const ClosestPose<T> result_in_path =
      FindSingleClosestInDefaultPath(lane, ego_pose, traffic_poses, scan_distance, side);
  if (path_or_branches == ScanStrategy::kPath) return result_in_path;

  const std::vector<LaneEndDistance<T>> distance_to_branches =
      FindConfluentBranches(lane, ego_pose, scan_distance, side);
  if (distance_to_branches.empty()) return result_in_path;

  // Find any leading traffic cars in lanes leading into the ego vehicle's
  // default path.
  const ClosestPose<T> result_in_branch =
      FindSingleClosestInBranches(lane, ego_pose, traffic_poses, distance_to_branches, scan_distance, side);

  if (result_in_path.distance <= result_in_branch.distance) {
    return result_in_path;
  }
  return result_in_branch;
}

template <typename T>
T TrafficPoseSelector<T>::GetSigmaVelocity(const RoadOdometry<T>& road_odometry) {
  DRAKE_THROW_UNLESS(road_odometry.lane != nullptr);
  DRAKE_THROW_UNLESS(IsWithinLaneBounds(road_odometry.lane, road_odometry.pos));
  return CalcSigmaVelocity(road_odometry.lane, road_odometry.pos, road_odometry.vel);
}

}  // namespace delphyne

// These instantiations must match the API documentation in
// traffic_pose_selector.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(class ::delphyne::TrafficPoseSelector)
