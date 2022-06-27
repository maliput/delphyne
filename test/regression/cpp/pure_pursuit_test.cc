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
#include "systems/pure_pursuit.h"

#include <gtest/gtest.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>

#include "delphyne/roads/road_builder.h"

namespace delphyne {
namespace {

static constexpr double kArcRadius = 25.;
static constexpr double kLinearTolerance{std::numeric_limits<double>::epsilon()};
static constexpr double kAngularTolerance{std::numeric_limits<double>::epsilon()};

using drake::AutoDiffXd;
using drake::Translation3;
using drake::Vector3;
using maliput::api::InertialPosition;
using maliput::api::RoadGeometryId;

class PurePursuitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a straight road with one lane.
    road_network_ =
        roads::CreateDragway("Single-Lane Dragway", 1 /* num_lanes */, 100 /* length */, 4. /* lane_width */,
                             0. /* shoulder_width */, 5. /* maximum_height */, kLinearTolerance, kAngularTolerance);
    road_ = road_network_->get()->road_geometry();
  }

  void MakeSemiCircleRoad() {
    const std::string multilane_description = R"R(
  maliput_multilane_builder:
    id: "single_lane_quarter_circle"
    lane_width: 4
    elevation_bounds: [0, 5]
    scale_length: 1.0
    linear_tolerance: .01
    angular_tolerance: 0.01
    computation_policy: prefer-speed
    right_shoulder: 0.
    left_shoulder: 0.
    points:
      start:
        xypoint: [0, 0, 0]  # x,y, heading
        zpoint: [0, 0, 0, 0]  # z, z_dot, theta (superelevation), theta_dot
    connections:
      0:
        lanes: [1, 0, 0]
        start: ["ref", "points.start.forward"]
        arc: [25, 180]
        explicit_end: ["ref", "points.start.forward"]
  )R";
    road_network_ = roads::CreateMultilaneFromDescription(multilane_description);
    road_ = road_network_->get()->road_geometry();
  }

  const PurePursuitParams<double> pp_params_{};
  const PurePursuitParams<AutoDiffXd> pp_params_ad_{};
  const SimpleCarParams<double> car_params_{};
  const SimpleCarParams<AutoDiffXd> car_params_ad_{};
  std::unique_ptr<delphyne::roads::RoadNetwork> road_network_;
  const maliput::api::RoadGeometry* road_{};
};

TEST_F(PurePursuitTest, Evaluate) {
  const maliput::api::Lane* const lane = road_->junction(0)->segment(0)->lane(0);

  // Set the ego car's pose to be all zeros, and facing parallel to the track in
  // the positive-s direction.
  drake::systems::rendering::PoseVector<double> ego_pose;
  ego_pose.set_translation(Eigen::Translation3d(0. /* s */, 0. /* r */, 0. /* h */));
  ego_pose.set_rotation(Eigen::Quaternion<double>::Identity());

  double result = PurePursuit<double>::Evaluate(pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be zero.
  EXPECT_EQ(0., result);

  // Introduce a positive r-offset  (car is to the left of the track).
  ego_pose.set_translation(Eigen::Translation3d(0. /* s */, 20. /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be between zero and -90-degrees.
  EXPECT_GT(0., result);
  EXPECT_LT(-M_PI_2, result);

  // Introduce a negative r-offset (car is to the right of the track).
  ego_pose.set_translation(Eigen::Translation3d(0. /* s */, -20. /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be between zero and 90-degrees.
  EXPECT_LT(0., result);
  EXPECT_GT(M_PI_2, result);

  // Introduce an r-offset much greater than the lookahead distance (typically
  // on the order of 10).
  ego_pose.set_translation(Eigen::Translation3d(0. /* s */, -1e5 /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be close to but not exceeding 90-degrees.
  EXPECT_NEAR(M_PI_2, result, 1e-2);
  EXPECT_GT(M_PI_2, result);

  // Arrange the pose to be to the right of the track and facing it at a
  // 90-degree angle.
  ego_pose.set_translation(Eigen::Translation3d(0. /* s */, -1. /* r */, 0. /* h */));
  const double yaw = M_PI_2;
  ego_pose.set_rotation(
      Eigen::Quaternion<double>(std::cos(yaw * 0.5) /* w */, 0. /* x */, 0. /* y */, std::sin(yaw * 0.5) /* z */));

  result = PurePursuit<double>::Evaluate(pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be between zero and 90-degrees.
  EXPECT_GT(0., result);
  EXPECT_LT(-M_PI_2, result);
}

// Tests that rotational symmetry is preserved.
TEST_F(PurePursuitTest, RotationalSymmetry) {
  MakeSemiCircleRoad();
  const maliput::api::Lane* const lane = road_->junction(0)->segment(0)->lane(0);

  // Situate the ego car at the START of the arc-shaped lane such that it is
  // aligned with the lane, but offset by r = -1 from the center-line.
  drake::systems::rendering::PoseVector<double> ego_pose;
  ego_pose.set_translation(Eigen::Translation3d(0. /* x */, -1. /* y */, 0. /* z */));
  ego_pose.set_rotation(Eigen::Quaternion<double>::Identity());

  double result_zero = PurePursuit<double>::Evaluate(pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  // Situate the ego car at the END of the arc-shaped lane such that it is
  // aligned with the lane, but offset by r = -1 from the center-line.
  ego_pose.set_translation(Eigen::Translation3d(kArcRadius + 1. /* x */, kArcRadius /* y */, 0. /* z */));
  const double yaw = M_PI_2;
  ego_pose.set_rotation(
      Eigen::Quaternion<double>(std::cos(yaw * 0.5) /* w */, 0. /* x */, 0. /* y */, std::sin(yaw * 0.5) /* z */));

  double result_pi_by_two = PurePursuit<double>::Evaluate(pp_params_, car_params_, {lane, true /* with_s */}, ego_pose);

  EXPECT_NEAR(result_zero, result_pi_by_two, 1e-6);
}

TEST_F(PurePursuitTest, EvaluateAutoDiff) {
  const maliput::api::Lane* const lane = road_->junction(0)->segment(0)->lane(0);

  // Set the ego car's pose to be all zeros, and facing parallel to the track in
  // the positive-s direction.
  // Derivatives are set such that ∂s/∂s = 1, ∂r/∂s = 0., ∂h/∂s = 0, etc.
  AutoDiffXd s(0., Vector3<double>(1., 0., 0.));
  AutoDiffXd r(0., Vector3<double>(0., 1., 0.));
  AutoDiffXd h(0., Vector3<double>(0., 0., 1.));
  AutoDiffXd ad_one(1., Vector3<double>(0., 0., 0.));
  AutoDiffXd ad_zero(0., Vector3<double>(0., 0., 0.));
  drake::systems::rendering::PoseVector<AutoDiffXd> ego_pose;
  ego_pose.set_translation(Translation3<AutoDiffXd>(s, r, h));
  ego_pose.set_rotation(
      Eigen::Quaternion<AutoDiffXd>(ad_one /* w */, ad_zero /* x */, ad_zero /* y */, ad_zero /* z */));

  AutoDiffXd result =
      PurePursuit<AutoDiffXd>::Evaluate(pp_params_ad_, car_params_ad_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be zero and its derivative to be negatively
  // related to changes in r, but insensitive to changes in s and h.
  EXPECT_EQ(0., result.value());
  EXPECT_EQ(0., result.derivatives()(0));  // ∂(steering_angle)/∂s
  EXPECT_GT(0., result.derivatives()(1));  // ∂(steering_angle)/∂r
  EXPECT_EQ(0., result.derivatives()(2));  // ∂(steering_angle)/∂h

  // Set the ego car to the left of the centerline, and facing parallel to the
  // track in the positive-s direction.
  r.value() = 1.;
  ego_pose.set_translation(Translation3<AutoDiffXd>(s, r, h));
  result = PurePursuit<AutoDiffXd>::Evaluate(pp_params_ad_, car_params_ad_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be negative and its derivative to be
  // negatively related to changes in r, but insensitive to changes in s and h.
  EXPECT_GT(0., result.value());
  EXPECT_EQ(0., result.derivatives()(0));  // ∂(steering_angle)/∂s
  EXPECT_GT(0., result.derivatives()(1));  // ∂(steering_angle)/∂r
  EXPECT_EQ(0., result.derivatives()(2));  // ∂(steering_angle)/∂h

  // Set the ego car to the right of the centerline, and facing parallel to the
  // track in the positive-s direction.
  r.value() = -1.;
  ego_pose.set_translation(Translation3<AutoDiffXd>(s, r, h));
  result = PurePursuit<AutoDiffXd>::Evaluate(pp_params_ad_, car_params_ad_, {lane, true /* with_s */}, ego_pose);

  // Expect the steering angle to be positive and its derivative to be
  // negatively related to changes in r, but insensitive to changes in s and h.
  EXPECT_LT(0., result.value());
  EXPECT_EQ(0., result.derivatives()(0));  // ∂(steering_angle)/∂s
  EXPECT_GT(0., result.derivatives()(1));  // ∂(steering_angle)/∂r
  EXPECT_EQ(0., result.derivatives()(2));  // ∂(steering_angle)/∂h
}

TEST_F(PurePursuitTest, ComputeGoalPoint) {
  // Set the ego car's pose to be all zeros, and facing parallel to the track in
  // the positive-s direction, with an r-offset.
  drake::systems::rendering::PoseVector<double> pose{};
  pose.set_translation(Eigen::Translation3d(50. /* s */, 5. /* r */, 0. /* h */));
  pose.set_rotation(Eigen::Quaternion<double>::Identity());
  const maliput::api::Lane* const lane = road_->junction(0)->segment(0)->lane(0);

  InertialPosition goal_position =
      PurePursuit<double>::ComputeGoalPoint(10. /* s_lookahead */, {lane, true /* with_s */}, pose);

  // Expect the goal point to lie on the lane ordinate.
  EXPECT_EQ(60., goal_position.x());
  EXPECT_EQ(0., goal_position.y());
  EXPECT_EQ(0., goal_position.z());

  // Flip the pose 180 degrees.
  goal_position = PurePursuit<double>::ComputeGoalPoint(10. /* s_lookahead */, {lane, false /* with_s */}, pose);

  // Expect the goal point to lie on the lane ordinate.
  EXPECT_EQ(40., goal_position.x());
  EXPECT_EQ(0., goal_position.y());
  EXPECT_EQ(0., goal_position.z());

  // Take the lookahead distance to be beyond the end of the lane.
  goal_position = PurePursuit<double>::ComputeGoalPoint(60. /* s_lookahead */, {lane, true /* with_s */}, pose);

  // Expect the result to saturate.
  EXPECT_EQ(100., goal_position.x());
  EXPECT_EQ(0., goal_position.y());
  EXPECT_EQ(0., goal_position.z());
}
// TODO(jadecastro): Test with curved lanes once
// multilane::Lane::ToRoadPosition() is implemented.

}  // namespace
}  // namespace delphyne
