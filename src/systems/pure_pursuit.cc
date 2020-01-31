// Copyright 2018 Toyota Research Institute

#include "systems/pure_pursuit.h"

#include <cmath>
#include <memory>

#include <drake/common/autodiff.h>
#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>
#include <drake/common/extract_double.h>
#include <drake/common/symbolic.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/saturate.h>
#include <maliput/api/lane.h>

namespace delphyne {

using drake::systems::rendering::PoseVector;
using maliput::api::GeoPosition;
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

  const GeoPosition goal_position = ComputeGoalPoint(pp_params.s_lookahead(), lane_direction, pose);

  const T x = pose.get_translation().translation().x();
  const T y = pose.get_translation().translation().y();
  const T heading = drake::math::RollPitchYaw<T>(pose.get_rotation()).yaw_angle();

  const T delta_r = -(goal_position.x() - x) * sin(heading) + (goal_position.y() - y) * cos(heading);
  const T curvature = 2. * delta_r / pow(pp_params.s_lookahead(), 2.);

  // Return the steering angle.
  return atan2(car_params.wheelbase() * curvature, T(1.));
  // N.B. atan2(*, 1) is used here in the absence of an atan() autodiff
  // overload.
}

template <typename T>
const GeoPosition PurePursuit<T>::ComputeGoalPoint(const T& s_lookahead, const LaneDirection& lane_direction,
                                                   const PoseVector<T>& pose) {
  const Lane* const lane = lane_direction.lane;
  const bool with_s = lane_direction.with_s;
  const LanePositionResult result =
      lane->ToLanePosition({drake::ExtractDoubleOrThrow(pose.get_isometry().translation().x()),
                            drake::ExtractDoubleOrThrow(pose.get_isometry().translation().y()),
                            drake::ExtractDoubleOrThrow(pose.get_isometry().translation().z())});
  const T s_new = with_s ? T(result.lane_position.s()) + s_lookahead : T(result.lane_position.s()) - s_lookahead;
  const T s_goal = drake::math::saturate(s_new, T(0.), T(lane->length()));
  // TODO(jadecastro): Add support for locating goal points in ongoing lanes.
  return lane->ToGeoPosition(
      {drake::ExtractDoubleOrThrow(s_goal), 0. * result.lane_position.r(), result.lane_position.h()});
}

}  // namespace delphyne

// These instantiations must match the API documentation in pure_pursuit.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(class delphyne::PurePursuit)
