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
#include <drake/systems/rendering/pose_vector.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>

namespace delphyne {

/// Given a PoseVector @p pose, find a car's current RoadGeometry via a search
/// of immediate ongoing lanes, starting with the current one.  Uses the
/// provided FrameVelocity @p velocity to determine which side of the lane
/// (provided in @p rp) to check.  Updates @p rp with the result, if one is
/// found; otherwise updates @p rp using the result of a global search of the @p
/// road.
///
/// Instantiated templates for the following scalar types `T` are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
//
// TODO(jadecastro) Enable symbolic::Expression (currently,
// PoseSelector::GetSigmaVelocity() is blocking us, as it uses
// ExtractDoubleOrThrow()).
template <typename T>
void CalcOngoingRoadPosition(const drake::systems::rendering::PoseVector<T>& pose,
                             const drake::systems::rendering::FrameVelocity<T>& velocity,
                             const maliput::api::RoadGeometry& road, maliput::api::RoadPosition* rp);

}  // namespace delphyne
