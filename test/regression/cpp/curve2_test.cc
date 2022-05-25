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
#include "systems/curve2.h"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace delphyne {
namespace {

typedef Curve2<double> Curve2d;
typedef Curve2d::Point2 Point2d;

// An empty curve.
GTEST_TEST(Curve2Test, EmptyTest) {
  const std::vector<Point2d> empty_waypoints{};
  const Curve2d empty_curve{empty_waypoints};
  EXPECT_DOUBLE_EQ(empty_curve.path_length(), 0.0);
}

// A single waypoint.
GTEST_TEST(Curve2Test, SingleWaypointTest) {
  const std::vector<Point2d> single_waypoint{
      Point2d{1.0, 2.0},
  };
  EXPECT_THROW(Curve2d{single_waypoint}, std::exception);
}

// A single segment.
GTEST_TEST(Curve2Test, BasicTest) {
  const Point2d start_point{1.0, 2.0};
  const Point2d end_point{2.0, 3.0};
  const std::vector<Point2d> segment_waypoints{start_point, end_point};
  const Curve2d segment{segment_waypoints};
  EXPECT_DOUBLE_EQ(segment.path_length(), M_SQRT2);
  auto waypoints = segment.waypoints();
  EXPECT_GE(static_cast<int>(waypoints.size()), 2);
  EXPECT_EQ(waypoints[0], start_point);
  EXPECT_EQ(waypoints[1], end_point);

  auto before_start = segment.GetPosition(-1.0);
  auto at_start = segment.GetPosition(0.0);
  auto at_end = segment.GetPosition(segment.path_length());
  auto after_end = segment.GetPosition(10.0);

  EXPECT_EQ(before_start.position, start_point);
  EXPECT_EQ(at_start.position, start_point);
  EXPECT_EQ(at_end.position, end_point);
  EXPECT_EQ(after_end.position, end_point);

  for (auto it : {before_start, at_start, at_end, after_end}) {
    EXPECT_DOUBLE_EQ(it.position_dot(0), M_SQRT1_2);
    EXPECT_DOUBLE_EQ(it.position_dot(1), M_SQRT1_2);
  }
}

}  // namespace
}  // namespace delphyne
