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

#include "systems/pure_pursuit.h"

#include <cmath>
#include <memory>

#include <drake/common/autodiff.h>
#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>
#include <drake/common/extract_double.h>
#include <drake/common/symbolic.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/saturate.h>
#include <maliput/api/lane.h>

namespace delphyne {

using drake::systems::rendering::PoseVector;
using maliput::api::InertialPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::LanePositionResult;

template <typename T>
T PurePursuit<T>::Evaluate(const PurePursuitParams<T>& pp_params, const SimpleCarParams<T>& car_params,
                           const LaneDirection& lane_direction, const PoseVector<T>& pose) {
  DRAKE_DEMAND(pp_params.IsValid());
  DRAKE_DEMAND(car_params.IsValid());

  using std::atan2;
  using std::cos;
  using std::pow;
  using std::sin;

  const InertialPosition goal_position = ComputeGoalPoint(pp_params.s_lookahead(), lane_direction, pose);

  const T x = pose.get_translation().translation().x();
  const T y = pose.get_translation().translation().y();
  const T heading = drake::math::RollPitchYaw<T>(pose.get_rotation()).yaw_angle();

  const T delta_r = -(T(goal_position.x()) - x) * sin(heading) + (T(goal_position.y()) - y) * cos(heading);
  const T curvature = 2. * delta_r / pow(pp_params.s_lookahead(), 2.);

  // Return the steering angle.
  return atan2(car_params.wheelbase() * curvature, T(1.));
  // N.B. atan2(*, 1) is used here in the absence of an atan() autodiff
  // overload.
}

template <typename T>
const InertialPosition PurePursuit<T>::ComputeGoalPoint(const T& s_lookahead, const LaneDirection& lane_direction,
                                                        const PoseVector<T>& pose) {
  const Lane* const lane = lane_direction.lane;
  const bool with_s = lane_direction.with_s;
  const LanePositionResult result =
      lane->ToLanePosition({drake::ExtractDoubleOrThrow(pose.get_transform().translation().x()),
                            drake::ExtractDoubleOrThrow(pose.get_transform().translation().y()),
                            drake::ExtractDoubleOrThrow(pose.get_transform().translation().z())});
  const T s_new = with_s ? T(result.lane_position.s()) + s_lookahead : T(result.lane_position.s()) - s_lookahead;
  const T s_goal = drake::math::saturate(s_new, T(0.), T(lane->length()));
  // TODO(jadecastro): Add support for locating goal points in ongoing lanes.
  return lane->ToInertialPosition(
      {drake::ExtractDoubleOrThrow(s_goal), 0. * result.lane_position.r(), result.lane_position.h()});
}

}  // namespace delphyne

// These instantiations must match the API documentation in pure_pursuit.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(class delphyne::PurePursuit)
