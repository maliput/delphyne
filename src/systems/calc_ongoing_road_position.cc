// Copyright 2018 Toyota Research Institute

#include "systems/calc_ongoing_road_position.h"

#include <drake/common/autodiff.h>
#include <drake/common/symbolic.h>
#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/segment.h>

#include "systems/traffic_pose_selector.h"

namespace delphyne {

using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;
using maliput::api::GeoPositionT;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::LanePositionResultT;
using maliput::api::LanePositionT;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;

template <typename T>
void CalcOngoingRoadPosition(const PoseVector<T>& pose, const FrameVelocity<T>& velocity, const RoadGeometry& road,
                             RoadPosition* rp) {
  DRAKE_THROW_UNLESS(rp != nullptr);
  const auto gp = GeoPositionT<T>::FromXyz(pose.get_isometry().translation());
  if (!rp->lane) {
    // Do an exhaustive search.
    *rp = road.ToRoadPosition(gp.MakeDouble()).road_position;
    return;
  }

  const double tol = rp->lane->segment()->junction()->road_geometry()->linear_tolerance();
  LanePositionResultT<double> lpr = rp->lane->ToLanePositionT<double>(gp.MakeDouble());
  LanePositionT<T> lpr_lane_position({lpr.lane_position.s(), lpr.lane_position.r(), lpr.lane_position.h()});
  if (lpr.distance <= tol) {  // Our current lane is good; just update position.
    rp->pos = lpr.lane_position;
    return;
  }

  // Check the ongoing lanes at the end corresponding to the direction the car
  // is moving.
  const T s_dot = TrafficPoseSelector<T>::GetSigmaVelocity({rp->lane, lpr_lane_position, velocity});
  for (const auto end : {LaneEnd::kStart, LaneEnd::kFinish}) {
    // Check only the relevant lane end.  If s_dot == 0, check both ends
    // (velocity isn't informative).
    if (s_dot > 0. && end == LaneEnd::kStart) continue;
    if (s_dot < 0. && end == LaneEnd::kFinish) continue;

    const LaneEndSet* branches = rp->lane->GetOngoingBranches(end);
    if (!branches) continue;
    for (int i{0}; i < branches->size(); ++i) {
      const LaneEnd lane_end = branches->get(i);
      lpr = lane_end.lane->ToLanePositionT<double>(gp.MakeDouble());
      if (lpr.distance <= tol) {  // Update both the lane and position.
        rp->pos = lpr.lane_position;
        rp->lane = lane_end.lane;
        return;
      }
    }
  }
  // Do an exhaustive search if none is found, using the given lane as a hint.
  *rp = road.ToRoadPosition(gp.MakeDouble(), *rp).road_position;
}

// These instantiations must match the API documentation in
// calc_ongoing_road_position.h.
template void CalcOngoingRoadPosition(const PoseVector<drake::AutoDiffXd>& pose,
                                      const FrameVelocity<drake::AutoDiffXd>& velocity, const RoadGeometry& road,
                                      RoadPosition* rp);
template void CalcOngoingRoadPosition(const PoseVector<double>& pose, const FrameVelocity<double>& velocity,
                                      const RoadGeometry& road, RoadPosition* rp);

}  // namespace delphyne
