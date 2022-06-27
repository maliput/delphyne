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
#include "systems/road_path.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>

#include "delphyne/roads/road_builder.h"
#include "test_utilities/eigen_matrix_compare.h"

namespace delphyne {
namespace {

using maliput::api::HBounds;
using maliput::api::InertialPosition;
using maliput::api::JunctionId;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;

using drake::Vector3;

// The length of the straight lane segment.
const double kStraightRoadLength{10};

// The arc radius, angular displacement, and length of the curved road segment.
const double kCurvedRoadRadius{10};
const double kCurvedRoadLength{kCurvedRoadRadius * M_PI_2};

const double kTotalRoadLength{kStraightRoadLength + kCurvedRoadLength};

// Build a road with two lanes in series.
// When `is_opposing` is true the second lane's direction is reversed.
std::unique_ptr<delphyne::roads::RoadNetwork> MakeTwoLaneRoad(bool is_opposing) {
  const std::string multilane_description = !is_opposing ? R"R(
    maliput_multilane_builder:
      id: "two_lane_stretch"
      lane_width: 4
      elevation_bounds: [0, 5]
      scale_length: 1.0
      linear_tolerance: 0.01
      angular_tolerance: 0.01
      computation_policy: prefer-accuracy
      right_shoulder: 2.
      left_shoulder: 2.
      points:
        start:
          xypoint: [0, 0, 0]
          zpoint: [0, 0, 0]
      connections:
        0_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "points.start.forward"]
          length: 10
          z_end: ["ref", [0, 0, 0]]
        1_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "connections.0_fwd.end.ref.forward"]
          arc: [10, 90]
          z_end: ["ref", [0, 0, 0]]
    )R"
                                                         :
                                                         R"R(
    maliput_multilane_builder:
      id: "two_lane_stretch_opposed"
      lane_width: 4
      elevation_bounds: [0, 5]
      scale_length: 1.0
      linear_tolerance: 0.01
      angular_tolerance: 0.01
      computation_policy: prefer-accuracy
      right_shoulder: 2.
      left_shoulder: 2.
      points:
        start:
          xypoint: [0, 0, 0]
          zpoint: [0, 0, 0]
        start_rev:
          xypoint: [20., 10, -90]
          zpoint: [0, 0, 0]
      connections:
        0_fwd:
          lanes: [1, 0, 0]
          start: ["ref", "points.start.forward"]
          length: 10
          z_end: ["ref", [0, 0, 0]]
        1_rev:
          lanes: [1, 0, 0]
          start: ["ref", "points.start_rev.forward"]
          arc: [10, -90]
          explicit_end: ["ref", "connections.0_fwd.end.ref.forward"]

    )R";
  return roads::CreateMultilaneFromDescription(multilane_description);
}

const Lane* GetLaneById(const RoadGeometry& road, const std::string& lane_id) {
  for (int i = 0; i < road.num_junctions(); ++i) {
    if (road.junction(i)->id() == JunctionId(lane_id)) {
      return road.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

static double path_radius(const Vector3<double> value) {
  Vector3<double> result;
  result << value(0) - kStraightRoadLength, value(1) - kCurvedRoadRadius, value(2);
  return result.norm();
}

// Tests the constructor given a sufficient number of points.
GTEST_TEST(IdmControllerTest, ConstructOpposingSegments) {
  const double kStepSize{0.5};
  // Instantiate a road with opposing segments.
  auto road_opposing = MakeTwoLaneRoad(true);
  // Start in the straight segment and progress in the positive-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_opposing->get()->road_geometry(), "j:0_fwd"), /* lane */
                    true);                                                          /* with_s */
  // Create a finely-discretized path with a sufficient number of segments to
  // cover the full length.
  const auto path = RoadPath<double>(initial_lane_dir, /* initial_lane_direction */
                                     kStepSize,        /* step_size */
                                     100);             /* num_breaks */
  ASSERT_LE(kTotalRoadLength, path.get_path().end_time() - path.get_path().start_time());

  // Expect the lane boundary values to match.
  Vector3<double> expected_value{};
  Vector3<double> actual_value{};
  expected_value << 0., 0., 0.;
  actual_value = path.get_path().value(0.);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-3));
  // N.B. Using tolerance of 1e-3 to account for possible interpolation errors.

  // Derive s-position of the straight road segment from the number of break
  // point steps taken to reach  kStraightRoadLength from the end of the road.
  const double straight_length{path.get_path().start_time(std::ceil(kStraightRoadLength / kStepSize))};
  expected_value << 10., 0., 0.;
  actual_value = path.get_path().value(straight_length);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-3));

  const double total_length{path.get_path().end_time()};
  expected_value << 20., 10., 0.;
  actual_value = path.get_path().value(total_length);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-3));

  // Pick a few arbitrary points on the curved section, expect them to trace the
  // arc, hence demonstrating the interpolation is working.
  actual_value = path.get_path().value(4. / 7. * kTotalRoadLength);
  EXPECT_NEAR(kCurvedRoadRadius, path_radius(actual_value), 1e-3);

  actual_value = path.get_path().value(5. / 7. * kTotalRoadLength);
  EXPECT_NEAR(kCurvedRoadRadius, path_radius(actual_value), 1e-3);

  actual_value = path.get_path().value(6. / 7. * kTotalRoadLength);
  EXPECT_NEAR(kCurvedRoadRadius, path_radius(actual_value), 1e-3);

  // Check that the number of segments created is well below the max
  // number specified.
  EXPECT_GT(1000, path.get_path().get_number_of_segments());
}

GTEST_TEST(IdmControllerTest, ConstructConfluentSegments) {
  const double kStepSize{0.5};
  // Instantiate a road with confluent segments.
  auto road_confluent = MakeTwoLaneRoad(false);
  // Start in the curved segment, and progress in the negative-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_confluent->get()->road_geometry(), "j:1_fwd"), /* lane */
                    false);                                                          /* with_s */
  // Create a finely-discretized path with a sufficient number of segments to
  // cover the full length.
  const auto path = RoadPath<double>(initial_lane_dir, /* initial_lane_direction */
                                     kStepSize,        /* step_size */
                                     100);             /* num_breaks */
  ASSERT_LE(kTotalRoadLength, path.get_path().end_time() - path.get_path().start_time());

  // Expect the lane boundary values to match.
  Vector3<double> expected_value{};
  Vector3<double> actual_value{};
  expected_value << 20., 10., 0.;
  actual_value = path.get_path().value(0.);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-3));

  double total_length{path.get_path().end_time()};
  // Derive s-position of the straight road segment from the number of break
  // point steps taken to reach kStraightRoadLength from the start of the road.
  double straight_length{path.get_path().end_time(std::ceil(kStraightRoadLength / kStepSize))};
  double curved_length{total_length - straight_length};
  expected_value << 10., 0., 0.;
  actual_value = path.get_path().value(curved_length);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-3));

  expected_value << 0., 0., 0.;
  actual_value = path.get_path().value(total_length);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-3));
}

}  // namespace
}  // namespace delphyne
