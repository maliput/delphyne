// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#pragma once

#include <drake/systems/rendering/frame_velocity.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>

namespace delphyne {

/// RoadOdometry contains the position of the vehicle with respect to a lane in
/// a road, along with its velocity vector in the world frame.
template <typename T>
struct RoadOdometry {
  /// Default constructor.
  RoadOdometry() = default;
  /// Fully-parameterized constructor.
  RoadOdometry(const maliput::api::RoadPosition& road_position,
               const drake::systems::rendering::FrameVelocity<double>& frame_velocity)
      : lane(road_position.lane), pos(road_position.pos), vel(frame_velocity) {}
  /// Fully-parameterized constructor that is T-supported.
  RoadOdometry(const maliput::api::Lane* lane_in, const maliput::api::LanePosition& lane_position,
               const drake::systems::rendering::FrameVelocity<T>& frame_velocity)
      : lane(lane_in), pos(lane_position), vel(frame_velocity) {}

  const maliput::api::Lane* lane{};
  maliput::api::LanePosition pos{};
  drake::systems::rendering::FrameVelocity<T> vel{};
};

}  // namespace delphyne
