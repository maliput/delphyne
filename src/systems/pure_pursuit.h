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

#include <drake/common/drake_copyable.h>
#include <drake/systems/rendering/pose_vector.h>
#include <maliput/api/lane_data.h>

#include "gen/pure_pursuit_params.h"
#include "gen/simple_car_params.h"
#include "systems/lane_direction.h"

namespace delphyne {

/// PurePursuit computes the required steering angle to achieve a goal point on
/// an continuous planar path.  The path represents as the set of `r = 0`
/// positions along a Maliput lane, and a goal point is selected as a
/// pre-defined lookahead distance along the path in the intended direction of
/// travel.  The algorithm outputs the steering angle required to guide the
/// vehicle toward the goal point based on its current position in global
/// coordinates.
///
/// See [1] and the corresponding .cc file for details on the algorithm.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// [1] Coulter, R. Implementation of the Pure Pursuit Path Tracking
///     Algorithm. Carnegie Mellon University, Pittsburgh, Pennsylvania, Jan
///     1990.
template <typename T>
class PurePursuit {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuit)
  PurePursuit() = delete;

  /// Evaluates the required steering angle in radians using the pure-pursuit
  /// method.  Assumes zero elevation and superelevation.
  ///
  /// @param pp_params contains the `lookahead_distance`, the distance along the
  /// path based on the closest position on the path to the vehicle.
  ///
  /// @param car_params contains the `wheelbase` of the vehicle.
  ///
  /// @param lane_direction is a LaneDirection containing a reference lane and
  /// the direction of travel along the positive-s coordinate.
  ///
  /// @param pose is the PoseVector for the ego vehicle.
  // TODO(jadecastro): Infer the direction of travel rather than require it.
  static T Evaluate(const PurePursuitParams<T>& pp_params, const SimpleCarParams<T>& car_params,
                    const LaneDirection& lane_direction, const drake::systems::rendering::PoseVector<T>& pose);

  /// Computes the goal point at a distance `s_lookahead` from the closest
  /// position on the curve in the intended direction of travel, and `with_s`
  /// and `pose` are the direction of travel and PoseVector for the ego vehicle.
  static const maliput::api::InertialPosition ComputeGoalPoint(const T& s_lookahead,
                                                               const LaneDirection& lane_direction,
                                                               const drake::systems::rendering::PoseVector<T>& pose);
};

}  // namespace delphyne
