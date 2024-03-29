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
#include "systems/traffic_pose_selector.h"

#include <drake/common/extract_double.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>

#include "delphyne/macros.h"
#include "delphyne/roads/road_builder.h"

namespace delphyne {
namespace {

using maliput::api::HBounds;
using maliput::api::InertialPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RBounds;
using maliput::api::RoadPosition;
using maliput::api::Rotation;

using drake::math::RigidTransform;
using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseBundle;
using drake::systems::rendering::PoseVector;

using drake::AutoDiffXd;
using drake::ExtractDoubleOrThrow;
using drake::Isometry3;
using drake::Translation3;
using drake::Vector3;

constexpr double kInf = std::numeric_limits<double>::infinity();

// Constants for Dragway tests.
constexpr double kDragwayLaneLength{100.};
constexpr double kDragwayLaneWidth{2.};

constexpr double kEgoSPosition{10.};
constexpr double kEgoRPosition{-0.5 * kDragwayLaneWidth};

constexpr double kJustAheadSPosition{31.};
constexpr double kFarAheadSPosition{35.};
constexpr double kJustBehindSPosition{7.};
constexpr double kFarBehindSPosition{3.};
constexpr double kTrafficXVelocity{27.};

constexpr int kNumDragwayTrafficCars{4};

// Indices for a PoseBundle object (used in Dragway tests).
constexpr int kJustAheadIndex{0};
constexpr int kFarAheadIndex{1};
constexpr int kJustBehindIndex{2};
constexpr int kFarBehindIndex{3};

// The length of a straight multilane segment.
constexpr double kRoadSegmentLength{15.};

static const Lane* GetLaneByLaneId(const maliput::api::RoadGeometry& road, const std::string& lane_id) {
  const Lane* lane = road.ById().GetLane(maliput::api::LaneId{lane_id});
  DELPHYNE_VALIDATE(lane != nullptr, std::runtime_error,
                    std::string("No matching lane whose name is <") + lane_id + std::string("> in the road network"));
  return lane;
}

class TrafficPoseSelectorDragwayTest : public ::testing::Test {
 protected:
  void MakeDragway(int num_lanes, double lane_length) {
    DRAKE_ASSERT(num_lanes >= 0);
    // Create a dragway with the specified number of lanes starting at `x = 0`
    // and centered at `y = 0`.
    road_network_ =
        roads::CreateDragway("Test Dragway", num_lanes, lane_length, kDragwayLaneWidth, 0. /* shoulder width */,
                             5. /* maximum_height */, std::numeric_limits<double>::epsilon() /* linear_tolerance */,
                             std::numeric_limits<double>::epsilon() /* angular_tolerance */);
    road_ = road_network_->road_geometry();
  }
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const maliput::api::RoadGeometry* road_{};
};

template <typename T>
static void SetDefaultDragwayPoses(PoseVector<T>* ego_pose, PoseBundle<T>* traffic_poses) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == kNumDragwayTrafficCars);
  DRAKE_DEMAND(kEgoSPosition > 0. && kDragwayLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kJustAheadSPosition > kEgoSPosition && kDragwayLaneLength > kJustAheadSPosition);
  DRAKE_DEMAND(kEgoSPosition > kJustBehindSPosition && kJustBehindSPosition > 0.);

  // Create poses for four traffic cars and one ego positioned in the right
  // lane, interspersed as follows:
  //
  //     Far Behind   Just Behind     Ego     Just Ahead   Far Ahead
  //   |------o------------o-----------o----------o------------o-------------|
  //  s=0     3            7           10         31           35           100
  ego_pose->set_translation(Translation3<T>(T(kEgoSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */));

  const Translation3<T> translation_far_ahead(T(kFarAheadSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */);
  FrameVelocity<T> velocity_far_ahead{};
  velocity_far_ahead.get_mutable_value() << T(0.) /* ωx */, T(0.) /* ωy */, T(0.) /* ωz */,
      T(kTrafficXVelocity) /* vx */, T(0.) /* vy */, T(0.) /* vz */;
  const Translation3<T> translation_just_ahead(T(kJustAheadSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */);
  const Translation3<T> translation_just_behind(T(kJustBehindSPosition) /* s */, T(kEgoRPosition) /* r */,
                                                T(0.) /* h */);
  const Translation3<T> translation_far_behind(T(kFarBehindSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */);
  traffic_poses->set_transform(kFarAheadIndex, RigidTransform<T>(translation_far_ahead.vector()));
  traffic_poses->set_velocity(kFarAheadIndex, velocity_far_ahead);
  traffic_poses->set_transform(kJustAheadIndex, RigidTransform<T>(translation_just_ahead.vector()));
  traffic_poses->set_transform(kJustBehindIndex, RigidTransform<T>(translation_just_behind.vector()));
  traffic_poses->set_transform(kFarBehindIndex, RigidTransform<T>(translation_far_behind.vector()));
}

// Sets the poses for one ego car and one traffic car, with the relative
// positions of each determined by the given s_offset an r_offset values.  The
// optional `yaw` argument determines the orientation of the ego car with
// respect to the x-axis.
template <typename T>
static void SetPoses(const T& s_offset, const T& r_offset, PoseVector<T>* ego_pose, PoseBundle<T>* traffic_poses,
                     const T& yaw = T(0.)) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 1);
  DRAKE_DEMAND(kEgoSPosition > 0. && kDragwayLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kJustAheadSPosition > kEgoSPosition && kDragwayLaneLength > kJustAheadSPosition);
  DRAKE_DEMAND(kEgoSPosition > kJustBehindSPosition && kJustBehindSPosition > 0.);

  // Create poses for one traffic car and one ego car.
  ego_pose->set_translation(Translation3<T>(T(kEgoSPosition) /* s */, T(kEgoRPosition) /* r */, T(0.) /* h */));
  const drake::math::RollPitchYaw<T> rpy(T(0.), T(0.), T(yaw));
  ego_pose->set_rotation(rpy.ToQuaternion());

  const Translation3<T> translation(T(kEgoSPosition) + s_offset /* s */, T(kEgoRPosition) + r_offset /* r */,
                                    T(0.) /* h */);
  FrameVelocity<T> traffic_velocity{};
  traffic_velocity.get_mutable_value() << T(0.) /* ωx */, T(0.) /* ωy */, T(0.) /* ωz */, T(kTrafficXVelocity) /* vx */,
      T(0.) /* vy */, T(0.) /* vz */;
  traffic_poses->set_transform(0, RigidTransform<T>(translation));
  traffic_poses->set_velocity(0, traffic_velocity);
}

// Returns the lane in the road associated with the provided pose.
template <typename T>
const Lane* get_lane(const PoseVector<T>& pose, const maliput::api::RoadGeometry& road) {
  const InertialPosition inertial_position{ExtractDoubleOrThrow(pose.get_translation().x()),
                                           ExtractDoubleOrThrow(pose.get_translation().y()),
                                           ExtractDoubleOrThrow(pose.get_translation().z())};
  return road.ToRoadPosition(inertial_position).road_position.lane;
}

TEST_F(TrafficPoseSelectorDragwayTest, TwoLaneDragway) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance shorter than the lane length.
  const double scan_ahead_distance = kDragwayLaneLength / 2.;
  {
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_), ego_pose, traffic_poses,
                                                     scan_ahead_distance, ScanStrategy::kPath);

    // Verifies that the ego car and traffic cars are on the road and that the
    // correct leading and trailing cars are identified.
    EXPECT_EQ(kJustAheadSPosition, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kJustBehindSPosition, closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kJustAheadSPosition - kEgoSPosition, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition - kJustBehindSPosition, closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  // Test that we get the same result when just the leading car is returned.
  const ClosestPose<double>& closest_pose = TrafficPoseSelector<double>::FindSingleClosestPose(
      get_lane(ego_pose, *road_), ego_pose, traffic_poses, scan_ahead_distance, AheadOrBehind::kAhead,
      ScanStrategy::kPath);
  EXPECT_EQ(kJustAheadSPosition, closest_pose.odometry.pos.s());
  EXPECT_EQ(kJustAheadSPosition - kEgoSPosition, closest_pose.distance);
  {
    // Peer into the adjacent lane to the left.
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
                                                     scan_ahead_distance, ScanStrategy::kPath);

    // Expect to see no cars in the left lane.
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(-kInf, closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  // Bump the "just ahead" car into the lane to the left.
  Isometry3<double> isometry_just_ahead = traffic_poses.get_transform(kJustAheadIndex).GetAsIsometry3();
  isometry_just_ahead.translation().y() += kDragwayLaneWidth;
  traffic_poses.set_transform(kJustAheadIndex, RigidTransform<double>(isometry_just_ahead));
  {
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_), ego_pose, traffic_poses,
                                                     scan_ahead_distance, ScanStrategy::kPath);

    // Expect the "far ahead" car to be identified and with the correct speed.
    EXPECT_EQ(kFarAheadSPosition, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kJustBehindSPosition, closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kFarAheadSPosition - kEgoSPosition, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition - kJustBehindSPosition, closest_poses.at(AheadOrBehind::kBehind).distance);
    for (int i{0}; i < closest_poses.at(AheadOrBehind::kAhead).odometry.vel.size(); ++i) {
      const double velocity = closest_poses.at(AheadOrBehind::kAhead).odometry.vel[i];
      if (i == 3) {
        EXPECT_EQ(kTrafficXVelocity, velocity);
      } else {
        EXPECT_EQ(0., velocity);
      }
    }
  }

  // Bump the "far ahead" car into the lane to the left.
  Isometry3<double> isometry_far_ahead = traffic_poses.get_transform(kFarAheadIndex).GetAsIsometry3();
  isometry_far_ahead.translation().y() += kDragwayLaneWidth;
  traffic_poses.set_transform(kFarAheadIndex, RigidTransform<double>(isometry_far_ahead));
  {
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_), ego_pose, traffic_poses,
                                                     scan_ahead_distance, ScanStrategy::kPath);

    // Looking forward, we expect there to be no car in sight.
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kAhead).distance);
    for (int i = 0; i < 6; ++i) {
      EXPECT_EQ(0., closest_poses.at(AheadOrBehind::kAhead).odometry.vel[i]);
      // N.B. Defaults to zero velocity.
    }
  }

  {
    // Peer into the adjacent lane to the left.
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
                                                     scan_ahead_distance, ScanStrategy::kPath);

    // Expect there to be no car behind on the immediate left and the "just
    // ahead" car to be leading.
    EXPECT_EQ(kJustAheadSPosition, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(-kInf, closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kJustAheadSPosition - kEgoSPosition, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kBehind).distance);
  }
}

// Verifies the result when using the analogous branch checking functions.
TEST_F(TrafficPoseSelectorDragwayTest, TwoLaneDragwayCheckBranches) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance shorter than the lane length.
  const double scan_ahead_distance = kDragwayLaneLength / 2.;
  {
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_), ego_pose, traffic_poses,
                                                     scan_ahead_distance, ScanStrategy::kBranches);

    // Verifies that the ego car and traffic cars are on the road and that the
    // correct leading and trailing cars are identified within the path of the
    // ego.
    EXPECT_EQ(kJustAheadSPosition, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kJustBehindSPosition, closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(kJustAheadSPosition - kEgoSPosition, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition - kJustBehindSPosition, closest_poses.at(AheadOrBehind::kBehind).distance);
  }
}

// Verifies that CalcLaneDirection returns the correct result when the ego
// vehicle's orientation is altered.
TEST_F(TrafficPoseSelectorDragwayTest, EgoOrientation) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance shorter than the lane length.
  const double scan_ahead_distance = kDragwayLaneLength / 2.;

  for (double yaw = -M_PI; yaw <= M_PI; yaw += 0.1) {
    // N.B. 0 corresponds to "aligned with the lane along the s-direction".
    drake::math::RollPitchYaw<double> rpy(0., 0., yaw);
    ego_pose.set_rotation(rpy.ToQuaternion());

    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_), ego_pose, traffic_poses,
                                                     scan_ahead_distance, ScanStrategy::kPath);

    // Expect the correct result independent of the ego vehicle's orientation.
    const bool is_with_s = yaw > -M_PI / 2. && yaw < M_PI / 2.;
    ClosestPose<double> closest_ahead =
        (is_with_s) ? closest_poses.at(AheadOrBehind::kAhead) : closest_poses.at(AheadOrBehind::kBehind);
    ClosestPose<double> closest_behind =
        (is_with_s) ? closest_poses.at(AheadOrBehind::kBehind) : closest_poses.at(AheadOrBehind::kAhead);
    EXPECT_EQ(kJustAheadSPosition, closest_ahead.odometry.pos.s());
    EXPECT_EQ(kJustBehindSPosition, closest_behind.odometry.pos.s());
  }
}

TEST_F(TrafficPoseSelectorDragwayTest, NoCarsOnShortRoad) {
  // When no cars are found on a dragway whose length is less than the
  // scan_distance, then infinite distances should be returned.
  const double kShortLaneLength{40.};
  MakeDragway(2 /* num lanes */, kShortLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(kNumDragwayTrafficCars);

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &traffic_poses);

  // Choose a scan-ahead distance greater than the lane length.
  const double scan_ahead_distance = kShortLaneLength + 10.;
  EXPECT_GT(scan_ahead_distance, kShortLaneLength - ego_pose.get_translation().x());
  EXPECT_GT(scan_ahead_distance, ego_pose.get_translation().x());

  // Scan for cars in the left lane, which should contain no cars.
  const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses = TrafficPoseSelector<double>::FindClosestPair(
      get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses, scan_ahead_distance, ScanStrategy::kPath);

  // Expect infinite distances.
  EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kAhead).distance);
  EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kBehind).distance);
}

// Verifies the result when the s-positions of the ego and traffic vehicles have
// the same s-position (side-by-side in adjacent lanes).
TEST_F(TrafficPoseSelectorDragwayTest, IdenticalSValues) {
  MakeDragway(2 /* num lanes */, kDragwayLaneLength);

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(1);

  // Create poses for one traffic car and one ego car positioned side-by-side,
  // with the ego vehicle in the right lane and the traffic vehicle in the left
  // lane.
  SetPoses(0. /* s_offset */, kDragwayLaneWidth /* r_offset */, &ego_pose, &traffic_poses);

  {
    // Peer into the adjacent lane to the left.
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
                                                     1000. /* scan_ahead_distance */, ScanStrategy::kPath);

    // Verifies that, if the cars are side-by-side, then the traffic car is
    // classified as a trailing car (and not the leading car).
    //
    // N.B. The dragway has a magic teleportation device at the end of each lane
    // that returns cars to the opposite end of the same lane.  The immediate
    // implication to TrafficPoseSelector is that cars located "behind" the ego
    // will be
    // also visible ahead of it, provided `scan_ahead_distance` is large enough.
    EXPECT_EQ(kEgoSPosition, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kDragwayLaneLength, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition, closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(0., closest_poses.at(AheadOrBehind::kBehind).distance);
  }

  {
    // Repeat the same computation, but with a myopic scan-ahead distance that
    // is much smaller than kDragwayLaneLength.
    const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses =
        TrafficPoseSelector<double>::FindClosestPair(get_lane(ego_pose, *road_)->to_left(), ego_pose, traffic_poses,
                                                     kDragwayLaneLength / 2. /* scan_ahead_distance */,
                                                     ScanStrategy::kPath);

    // Verifies that no traffic car is seen ahead.
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kAhead).odometry.pos.s());
    EXPECT_EQ(kInf, closest_poses.at(AheadOrBehind::kAhead).distance);
    EXPECT_EQ(kEgoSPosition, closest_poses.at(AheadOrBehind::kBehind).odometry.pos.s());
    EXPECT_EQ(0., closest_poses.at(AheadOrBehind::kBehind).distance);
  }
}

TEST_F(TrafficPoseSelectorDragwayTest, TestGetSigmaVelocity) {
  MakeDragway(1 /* num lanes */, kDragwayLaneLength);

  // Provide GetSigmaVelocity() with a null Lane.
  RoadPosition null_rp(nullptr, maliput::api::LanePosition(0., 0., 0.));
  FrameVelocity<double> velocity{};

  // Expect it to throw.
  EXPECT_THROW(TrafficPoseSelector<double>::GetSigmaVelocity({null_rp, velocity}), std::runtime_error);

  // Set a valid lane.
  const Lane* lane = road_->junction(0)->segment(0)->lane(0);
  RoadPosition position(lane, maliput::api::LanePosition(0., 0., 0.));

  // Expect the s-velocity to be zero.
  double sigma_v = TrafficPoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_EQ(0., sigma_v);

  // Set the velocity to be along the lane's s-coordinate.
  velocity[3] = 10.;
  // Expect the s-velocity to match.
  sigma_v = TrafficPoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_EQ(10., sigma_v);

  // Set a velocity vector at 45-degrees with the lane's s-coordinate.
  velocity[3] = 10. * std::cos(M_PI / 4.);
  velocity[4] = 10. * std::sin(M_PI / 4.);
  // Expect the s-velocity to be attenuated by sqrt(2) / 2.
  sigma_v = TrafficPoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_NEAR(10. * std::sqrt(2.) / 2., sigma_v, 1e-12);

  // Verifies the consistency of the result when the s-value is set to
  // end-of-lane.
  position.pos.set_s(lane->length());
  sigma_v = TrafficPoseSelector<double>::GetSigmaVelocity({position, velocity});
  EXPECT_NEAR(10. * std::sqrt(2.) / 2., sigma_v, 1e-12);
}

// Build a road with three lanes in series.  If `is_opposing` is true, then the
// middle segment is reversed.
std::unique_ptr<maliput::api::RoadNetwork> MakeThreeSegmentRoad(bool is_opposing) {
  const std::string multilane_description = !is_opposing ? R"R(
    maliput_multilane_builder:
      id: "three_lane_stretch_forward"
      lane_width: 2
      elevation_bounds: [0, 5]
      scale_length: 1.0
      linear_tolerance: 0.01
      angular_tolerance: 0.01
      computation_policy: prefer-accuracy
      right_shoulder: 0.
      left_shoulder: 0.
      points:
        start:
          xypoint: [0, 0, 0]
          zpoint: [0, 0, 0]
      connections:
        0_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "points.start.forward"]
          length: 15
          z_end: ["ref", [0, 0, 0]]
        1_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "connections.0_fwd.end.ref.forward"]
          length: 15
          z_end: ["ref", [0, 0, 0]]
        2_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "connections.1_fwd.end.ref.forward"]
          length: 15
          z_end: ["ref", [0, 0, 0]]

      )R"
                                                         :
                                                         R"R(
    maliput_multilane_builder:
      id: "three_lane_stretch_forward_opposed"
      lane_width: 2
      elevation_bounds: [0, 5]
      scale_length: 1.0
      linear_tolerance: 0.01
      angular_tolerance: 0.01
      computation_policy: prefer-accuracy
      right_shoulder: 0.
      left_shoulder: 0.
      points:
        start:
          xypoint: [0, 0, 0]
          zpoint: [0, 0, 0]
        start_rev:
          xypoint: [30, 0, 0]
          zpoint: [0, 0, 0]
      connections:
        0_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "points.start.forward"]
          length: 15
          z_end: ["ref", [0, 0, 0]]
        1_rev:
          lanes: [1, 0, 0]
          start: ["ref", "points.start_rev.reverse"]
          length: 15
          explicit_end: ["ref", "connections.0_fwd.end.ref.reverse"]
        2_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "connections.1_rev.start.ref.reverse"]
          length: 15
          z_end: ["ref", [0, 0, 0]]

    )R";
  return roads::CreateMultilaneFromDescription(multilane_description);
}

// Verifies the soundness of the results when applied to multi-segment roads.
GTEST_TEST(TrafficPoseSelectorTest, MultiSegmentRoad) {
  // Instantiate multilane roads with multiple segments.
  std::vector<const maliput::api::RoadGeometry*> roads;
  const auto road_network_1 = MakeThreeSegmentRoad(false);
  const auto road_network_2 = MakeThreeSegmentRoad(true);
  roads.push_back(road_network_1->road_geometry());  // Road with consistent
                                                     // with_s
                                                     // directionality.
  roads.push_back(road_network_2->road_geometry());  // Road constructed with
                                                     // alternating with_s.

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(1);

  // Choose a scan-ahead distance at least as long as the entire road.
  const double scan_ahead_distance = 3. * kRoadSegmentLength;

  for (const auto& road : roads) {
    // At each iteration, increment the traffic car's position ahead through
    // each lane.
    for (double s_offset = 8.; s_offset <= 3. * kRoadSegmentLength - kEgoSPosition; s_offset += 5.) {
      // Situate the ego car within the 0th segment, facing along the x-axis,
      // along with a traffic car that is in front of the ego car by an amount
      // `s_offset`.
      SetPoses(s_offset, 0. /* r_offset */, &ego_pose, &traffic_poses, 0. /* yaw angle */);

      // Determine the distance to the car ahead the ego car.
      const ClosestPose<double> closest_pose_ahead = TrafficPoseSelector<double>::FindSingleClosestPose(
          get_lane(ego_pose, *road), ego_pose, traffic_poses, scan_ahead_distance, AheadOrBehind::kAhead,
          ScanStrategy::kBranches);

      // Expect the detected distance to be the offset distance.
      EXPECT_EQ(s_offset, closest_pose_ahead.distance);

      // Situate the ego car within the 0th segment, facing against the x-axis.
      SetPoses(s_offset, 0. /* r_offset */, &ego_pose, &traffic_poses, M_PI /* yaw angle */);

      // Determine the distance to the car behind the ego car.
      const ClosestPose<double> closest_pose_behind = TrafficPoseSelector<double>::FindSingleClosestPose(
          get_lane(ego_pose, *road), ego_pose, traffic_poses, scan_ahead_distance, AheadOrBehind::kBehind,
          ScanStrategy::kPath);

      // Expect the detected distance to be the offset distance.
      EXPECT_EQ(s_offset, closest_pose_behind.distance);
    }
  }
}

// Construct a multilane road with three confluent feeder lanes corresponding to
// three distinct branch points.
std::unique_ptr<maliput::api::RoadNetwork> BuildOnrampRoad() {
  const std::string multilane_description = R"R(
    maliput_multilane_builder:
      id: "on_ramp_road"
      lane_width: 4.
      elevation_bounds: [0, 5]
      scale_length: 1.0
      linear_tolerance: 0.001
      angular_tolerance: 0.001
      computation_policy: prefer-accuracy
      right_shoulder: 2.
      left_shoulder: 2.
      points:
        start:
          xypoint: [0, 0, 0]
          zpoint: [0, 0, 0]
      connections:
        lane6:
          lanes: [1, 0, 0]
          start: ["ref", "points.start.forward"]
          arc: [25, -91.6732]
          z_end: ["ref", [0, 0, 0]]
        lane5:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane6.end.ref.forward"]
          arc: [25, 91.6732]
          z_end: ["ref", [0, 0, 0]]
        lane4:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane5.end.ref.forward"]
          arc: [25, -91.6732]
          z_end: ["ref", [0, 0, 0]]
        lane3:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane4.end.ref.forward"]
          arc: [25, 91.6732]
          z_end: ["ref", [0, 0, 0]]
        lane2:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane3.end.ref.forward"]
          arc: [25, -91.6732]
          z_end: ["ref", [0, 0, 0]]
        lane1:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane2.end.ref.forward"]
          arc: [25, 91.6732]
          z_end: ["ref", [0, 0, 0]]
        lane0:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane1.end.ref.forward"]
          length: 100
          z_end: ["ref", [0, 0, 0]]
        b0_lane1:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane1.end.ref.forward"]
          arc: [35, 81.8511135902388]
          z_end: ["ref", [0, 0, 0]]
        b0_lane0:
          lanes: [1, 0, 0]
          start: ["ref", "connections.b0_lane1.end.ref.forward"]
          length: 100
          z_end: ["ref", [0, 0, 0]]
        b1_lane1:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane3.end.ref.forward"]
          arc: [35, 81.8511135902388]
          z_end: ["ref", [0, 0, 0]]
        b1_lane0:
          lanes: [1, 0, 0]
          start: ["ref", "connections.b1_lane1.end.ref.forward"]
          length: 100
          z_end: ["ref", [0, 0, 0]]
        b2_lane1:
          lanes: [1, 0, 0]
          start: ["ref", "connections.lane5.end.ref.forward"]
          arc: [35, 81.8511135902388]
          z_end: ["ref", [0, 0, 0]]
        b2_lane0:
          lanes: [1, 0, 0]
          start: ["ref", "connections.b2_lane1.end.ref.forward"]
          length: 100
          z_end: ["ref", [0, 0, 0]]
      )R";
  return roads::CreateMultilaneFromDescription(multilane_description);
}

enum class LanePolarity { kWithS, kAgainstS };

// Appends a traffic car's pose to the provided traffic_poses.
void AddToTrafficPosesAt(int index, const Lane* traffic_lane, double traffic_s_position, double traffic_speed,
                         LanePolarity traffic_polarity, PoseBundle<double>* traffic_poses) {
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  const LanePosition srh{traffic_s_position, 0., 0.};
  const InertialPosition traffic_xyz = traffic_lane->ToInertialPosition(srh);
  const Eigen::Vector3d translation_ahead(traffic_xyz.x(), traffic_xyz.y(), traffic_xyz.z());
  isometry.translate(translation_ahead);

  const Rotation traffic_rotation = traffic_lane->GetOrientation(srh);
  maliput::math::Vector3 rpy = traffic_rotation.rpy().vector();
  rpy.x() = (traffic_polarity == LanePolarity::kWithS) ? rpy.x() : -rpy.x();
  rpy.y() = (traffic_polarity == LanePolarity::kWithS) ? rpy.y() : -rpy.y();
  rpy.z() -= (traffic_polarity == LanePolarity::kWithS) ? 0. : M_PI;
  const drake::math::RollPitchYaw<double> roll_pitch_yaw(rpy.x(), rpy.y(), rpy.z());
  isometry.rotate(roll_pitch_yaw.ToQuaternion());

  traffic_poses->set_transform(index, RigidTransform<double>(isometry));

  FrameVelocity<double> velocity_ahead{};
  velocity_ahead.get_mutable_value().head(3) = Vector3<double>::Zero();  // ω
  const drake::math::RotationMatrix<double> traffic_rotmat(roll_pitch_yaw);
  velocity_ahead.get_mutable_value().tail(3) = traffic_speed * traffic_rotmat.matrix().leftCols(1);  // v
  traffic_poses->set_velocity(index, velocity_ahead);
}

void SetDefaultOnrampPoses(const Lane* ego_lane, const Lane* traffic_lane, double traffic_speed, double ego_speed,
                           PoseVector<double>* ego_pose, FrameVelocity<double>* ego_velocity,
                           PoseBundle<double>* traffic_poses, LanePolarity ego_polarity,
                           LanePolarity traffic_polarity) {
  // Set the ego vehicle at s = 1. in the ego_lane.
  const LanePosition srh_near_start{1., 0., 0.};
  const InertialPosition ego_xyz = ego_lane->ToInertialPosition(srh_near_start);
  ego_pose->set_translation(Eigen::Translation3d(ego_xyz.x(), ego_xyz.y(), ego_xyz.z()));
  const Rotation ego_rotation = ego_lane->GetOrientation(srh_near_start);
  const double ego_roll = (ego_polarity == LanePolarity::kWithS) ? ego_rotation.roll() : -ego_rotation.roll();
  const double ego_pitch = (ego_polarity == LanePolarity::kWithS) ? ego_rotation.pitch() : -ego_rotation.pitch();
  const double ego_yaw = ego_rotation.yaw() - ((ego_polarity == LanePolarity::kWithS) ? 0. : M_PI);
  const drake::math::RollPitchYaw<double> new_rpy(ego_roll, ego_pitch, ego_yaw);
  ego_pose->set_rotation(new_rpy.ToQuaternion());

  const drake::math::RotationMatrix<double> ego_rotmat(new_rpy);
  drake::Vector6<double> velocity{};
  velocity.head(3) = Vector3<double>::Zero();                      // ω
  velocity.tail(3) = ego_speed * ego_rotmat.matrix().leftCols(1);  // v
  ego_velocity->set_velocity(drake::multibody::SpatialVelocity<double>(velocity));

  // Set the traffic car at s = Lane::length() - 1 in the traffic_lane.
  AddToTrafficPosesAt(0, traffic_lane, traffic_lane->length() - 1., traffic_speed, traffic_polarity, traffic_poses);
}

using Cases = std::map<LanePolarity, std::pair<AheadOrBehind, AheadOrBehind>>;

void CheckOnrampPosesInBranches(const maliput::api::RoadGeometry& road, const PoseVector<double>& ego_pose,
                                const PoseBundle<double>& traffic_poses, std::string expected_traffic_lane,
                                double expected_s_position, double expected_distance, LanePolarity ego_polarity,
                                const Cases& ego_cases) {
  const InertialPosition ego_inertial_position{ego_pose.get_translation().x(), ego_pose.get_translation().y(),
                                               ego_pose.get_translation().z()};
  const RoadPosition& ego_position = road.ToRoadPosition(ego_inertial_position).road_position;

  ClosestPose<double> closest_pose_leading = TrafficPoseSelector<double>::FindSingleClosestPose(
      ego_position.lane, ego_pose, traffic_poses, 1000. /* scan_ahead_distance */, ego_cases.at(ego_polarity).first,
      ScanStrategy::kBranches);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_EQ(expected_traffic_lane, closest_pose_leading.odometry.lane->id().string());
  if (expected_distance == kInf) {
    EXPECT_EQ(-kInf, closest_pose_leading.odometry.pos.s());
    EXPECT_EQ(kInf, closest_pose_leading.distance);
  } else {
    EXPECT_NEAR(expected_s_position, closest_pose_leading.odometry.pos.s(), 1e-3);
    EXPECT_NEAR(expected_distance, closest_pose_leading.distance, 1e-3);
  }

  const std::map<AheadOrBehind, const ClosestPose<double>> closest_poses = TrafficPoseSelector<double>::FindClosestPair(
      ego_position.lane, ego_pose, traffic_poses, 1000. /* scan_ahead_distance */, ScanStrategy::kBranches);

  // Verifies that the kAhead closest pose agrees with closest_pose_leading.
  const AheadOrBehind ego_view_1 = ego_cases.at(ego_polarity).first;
  EXPECT_EQ(closest_pose_leading.odometry.lane->id(), closest_poses.at(ego_view_1).odometry.lane->id());
  EXPECT_EQ(closest_pose_leading.odometry.pos.s(), closest_poses.at(ego_view_1).odometry.pos.s());
  EXPECT_EQ(closest_pose_leading.distance, closest_poses.at(ego_view_1).distance);
  // Verifies that the kBehind closest pose is infinity in the ego car's lane.
  const AheadOrBehind ego_view_2 = ego_cases.at(ego_polarity).second;
  EXPECT_EQ(ego_position.lane->id(), closest_poses.at(ego_view_2).odometry.lane->id());
  EXPECT_EQ(kInf, closest_poses.at(ego_view_2).odometry.pos.s());
  EXPECT_EQ(kInf, closest_poses.at(ego_view_2).distance);

  // TODO(jadecastro) Include tests at various velocities.
}

GTEST_TEST(TrafficPoseSelectorOnrampTest, CheckBranches) {
  const auto road_network = BuildOnrampRoad();
  const maliput::api::RoadGeometry* road = road_network->road_geometry();

  PoseVector<double> ego_pose;
  FrameVelocity<double> ego_velocity;
  PoseBundle<double> traffic_poses(1);

  struct TestCase {
    std::string ego_lane;
    std::string traffic_lane;
    LanePolarity traffic_orientation;
    double expected_distance;
    std::string expected_traffic_lane;
  };
  const std::vector<TestCase> test_cases{
      {"l:b0_lane0_0", "l:lane6_0", LanePolarity::kWithS, 252., "l:lane6_0"},
      {"l:b0_lane0_0", "l:lane0_0", LanePolarity::kWithS, kInf, "l:b0_lane0_0"},
      {"l:b1_lane0_0", "l:lane2_0", LanePolarity::kAgainstS, 12., "l:lane2_0"},
      {"l:b1_lane0_0", "l:lane2_0", LanePolarity::kWithS, kInf, "l:b1_lane0_0"},
      {"l:b2_lane0_0", "l:lane4_0", LanePolarity::kAgainstS, 12., "l:lane4_0"},
      {"l:b2_lane0_0", "l:lane4_0", LanePolarity::kWithS, kInf, "l:b2_lane0_0"},
      {"l:lane0_0", "l:b0_lane0_0", LanePolarity::kAgainstS, kInf, "l:lane0_0"},
      {"l:lane0_0", "l:b1_lane0_0", LanePolarity::kAgainstS, kInf, "l:lane0_0"},
      {"l:lane0_0", "l:b1_lane1_0", LanePolarity::kAgainstS, 32., "l:b1_lane1_0"},
      {"l:lane0_0", "l:b2_lane0_0", LanePolarity::kAgainstS, 12., "l:b2_lane0_0"},
      {"l:lane0_0", "l:b2_lane1_0", LanePolarity::kAgainstS, 112., "l:b2_lane1_0"},
      {"l:lane1_0", "l:b2_lane1_0", LanePolarity::kAgainstS, 72., "l:b2_lane1_0"},
  };

  // Define appropriate tests based on the ego car's LanePolarity.
  Cases ego_cases;
  ego_cases[LanePolarity::kWithS] = std::make_pair(AheadOrBehind::kBehind, AheadOrBehind::kAhead);
  ego_cases[LanePolarity::kAgainstS] = std::make_pair(AheadOrBehind::kAhead, AheadOrBehind::kBehind);

  for (const auto& it : test_cases) {
    const Lane* traffic_lane = GetLaneByLaneId(*road, it.traffic_lane);
    for (const auto ego_polarity : {LanePolarity::kWithS, LanePolarity::kAgainstS}) {
      SetDefaultOnrampPoses(GetLaneByLaneId(*road, it.ego_lane), traffic_lane, 10. /* traffic_speed */,
                            10. /* ego_speed */, &ego_pose, &ego_velocity, &traffic_poses, ego_polarity,
                            it.traffic_orientation);
      // TODO(jadecastro) Include additional unit tests at varying ego/traffic
      // speeds once the code accepts externally-defined ego velocities.
      CheckOnrampPosesInBranches(*road, ego_pose, traffic_poses, it.expected_traffic_lane, traffic_lane->length() - 1.,
                                 /* expected s-position */
                                 it.expected_distance, ego_polarity, ego_cases);
    }
  }

  // Checks that behavior is as expected when two traffic cars are introduced.
  PoseBundle<double> two_traffic_poses(2);
  const TestCase test_case = test_cases[2];  // Ego in b1_lane0, Traffic in
                                             // lane2, facing kAgainstS.
  const Lane* traffic_lane = GetLaneByLaneId(*road, test_case.traffic_lane);
  for (const auto ego_polarity : {LanePolarity::kWithS, LanePolarity::kAgainstS}) {
    SetDefaultOnrampPoses(GetLaneByLaneId(*road, test_case.ego_lane), traffic_lane, 10. /* traffic speed */,
                          10. /* ego_speed */, &ego_pose, &ego_velocity, &two_traffic_poses, ego_polarity,
                          test_case.traffic_orientation);

    // Add an additional car in b1_lane1, closer to the branch point than the
    // ego, and further than 12 meters from the ego.
    const Lane* other_traffic_lane = GetLaneByLaneId(*road, "l:b1_lane1_0");
    AddToTrafficPosesAt(1, other_traffic_lane, 10. /* other traffic s-position */, 10. /* other traffic speed */,
                        LanePolarity::kWithS, &two_traffic_poses);

    // Expect the traffic car in the branch lane to be selected (it is further
    // from the merge-point).
    const Lane* expected_lane = GetLaneByLaneId(*road, test_case.expected_traffic_lane);
    CheckOnrampPosesInBranches(*road, ego_pose, two_traffic_poses, test_case.expected_traffic_lane,
                               expected_lane->length() - 1.,
                               /* expected s-position */
                               test_case.expected_distance, ego_polarity, ego_cases);
  }
}

// TODO(jadecastro) We cannot yet test against AutoDiff for multi-segment roads
// because only the Dragway backend has AutoDiff implementations for
// Lane::ToLanePosition() and Lane::ToInertialPosition().

}  // namespace
}  // namespace delphyne
