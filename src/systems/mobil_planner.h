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

#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <drake/systems/rendering/pose_vector.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>

#include "gen/idm_planner_parameters.h"
#include "gen/mobil_planner_parameters.h"
#include "systems/calc_ongoing_road_position.h"
#include "systems/idm_planner.h"
#include "systems/lane_direction.h"
#include "systems/road_odometry.h"
#include "systems/traffic_pose_selector.h"

namespace delphyne {

/// MOBIL (Minimizing Overall Braking Induced by Lane Changes) [1] is a planner
/// that minimizes braking requirement for the ego car while also minimizing
/// (per a weighting factor) the braking requirements of any trailing cars
/// within the ego car's immediate neighborhood. The weighting factor
/// encapsulates the politeness of the ego car to the surrounding traffic.
/// Neighboring cars are defined as those cars immediately ahead and behind the
/// ego, in the current lane and any adjacent lanes; these are determined from
/// the PoseSelector logic applied to a multi-lane Maliput road.
///
/// The induced braking by the ego car and the car following immediately behind
/// it is compared with the induced braking by the ego and its new follower if
/// the ego were to move to any of the neighboring lanes. The choice that
/// minimizes the induced braking - alternatively maximizes the ego car's
/// "incentive" (the weighted sum of accelerations that the ego car and its
/// neighbors gain by changing lanes) - is chosen as the new lane request. The
/// request is expressed as a LaneDirection, that references a valid lane in the
/// provided RoadGeometry and the direction of travel.
///
/// Assumptions:
///   1) The planner supports only symmetric lane change rules, without giving
///      preference to lanes to the left or right.
///   2) The planner assumes all traffic behaves according to the Intelligent
///      Driver Model (IDM).
///   3) All neighboring lanes are confluent (i.e. with_s points in the same
///      direction).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: A PoseVector for the ego car.
///   (InputPort getter: ego_pose_input())
///
/// Input Port 1: A FrameVelocity for the ego car.
///   (InputPort getter: ego_velocity_input())
///
/// Input Port 2: A BasicVector containing the ego car's commanded acceleration
///   value intercepted from the vehicle's controller (e.g. IdmController).
///   (InputPort getter: ego_acceleration_input())
///
/// Input Port 3: A PoseBundle for the traffic cars, possibly including the ego
///   car's pose.
///   (InputPort getter: traffic_input())
///
/// Output Port 0: A LaneDirection containing a lane that the ego vehicle must
///   move into and the direction of travel with respect to the lane's canonical
///   direction of travel. LaneDirection must be consistent with the provided
///   road.
///   (OutputPort getter: lane_output())
///
/// @ingroup automotive_controllers
///
/// [1] Arne Kesting, Martin Treiber and Dirk Helbing, MOBIL: General
///     Lane-Changing Model for Car-Following Models, Journal of the
///     Transportation Research Board, v1999, 2007, pp 86-94.
///     http://trrjournalonline.trb.org/doi/abs/10.3141/1999-10.
template <typename T>
class MobilPlanner : public drake::systems::LeafSystem<T> {
 public:
  typedef typename std::map<AheadOrBehind, const ClosestPose<T>> ClosestPoses;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilPlanner)

  /// A constructor that initializes the MOBIL planner.
  /// @param road The pre-defined RoadGeometry.
  /// @param initial_with_s The initial direction of travel in the lane
  /// corresponding to the ego vehicle's initial state.
  /// @param road_position_strategy Determines whether or not to memorize
  /// RoadPosition. See `calc_ongoing_road_position.h`.
  /// @param period_sec The update period to use if road_position_strategy ==
  /// RoadPositionStrategy::kCache.
  MobilPlanner(const maliput::api::RoadGeometry& road, bool initial_with_s, RoadPositionStrategy road_position_strategy,
               double period_sec);

  /// See the class description for details on the following input ports.
  /// @{
  const drake::systems::InputPort<T>& ego_pose_input() const;
  const drake::systems::InputPort<T>& ego_velocity_input() const;
  const drake::systems::InputPort<T>& ego_acceleration_input() const;
  const drake::systems::InputPort<T>& traffic_input() const;
  const drake::systems::OutputPort<T>& lane_output() const;
  /// @}

  /// Getters to mutable named-vector references associated with MobilPlanner's
  /// Parameters groups.
  /// @{
  inline IdmPlannerParameters<T>& get_mutable_idm_params(drake::systems::Context<T>* context) const {
    return this->template GetMutableNumericParameter<IdmPlannerParameters>(context, kIdmParamsIndex);
  }
  inline MobilPlannerParameters<T>& get_mutable_mobil_params(drake::systems::Context<T>* context) const {
    return this->template GetMutableNumericParameter<MobilPlannerParameters>(context, kMobilParamsIndex);
  }
  /// @}

 protected:
  void DoCalcUnrestrictedUpdate(const drake::systems::Context<T>& context,
                                const std::vector<const drake::systems::UnrestrictedUpdateEvent<T>*>&,
                                drake::systems::State<T>* state) const override;

 private:
  void CalcLaneDirection(const drake::systems::Context<T>& context, LaneDirection* lane_direction) const;

  // Performs the calculations for the lane_output() port.
  void ImplCalcLaneDirection(const drake::systems::rendering::PoseVector<T>& ego_pose,
                             const drake::systems::rendering::FrameVelocity<T>& ego_velocity,
                             const drake::systems::rendering::PoseBundle<T>& traffic_poses,
                             const drake::systems::BasicVector<T>& ego_accel_command,
                             const IdmPlannerParameters<T>& idm_params, const MobilPlannerParameters<T>& mobil_params,
                             const maliput::api::RoadPosition& ego_rp, LaneDirection* lane_direction) const;

  // Computes a pair of incentive measures for the provided neighboring lanes.
  // The first and second elements in `lanes` correspond to, respectively, a
  // pair of lanes included in the incentive query. The respective incentives
  // for these lanes are returned as the first and second elements in the return
  // value.
  const std::pair<T, T> ComputeIncentives(const std::pair<const maliput::api::Lane*, const maliput::api::Lane*> lanes,
                                          const IdmPlannerParameters<T>& idm_params,
                                          const MobilPlannerParameters<T>& mobil_params,
                                          const ClosestPose<T>& ego_closest_pose,
                                          const drake::systems::rendering::PoseVector<T>& ego_pose,
                                          const drake::systems::rendering::PoseBundle<T>& traffic_poses,
                                          const T& ego_acceleration) const;

  // Computes a pair of incentive measures that consider the leading and
  // trailing vehicles that are closest to the pre-computed result in the
  // current lane. `closest_poses` contains the odometries and relative
  // distances to the leading and trailing cars.
  void ComputeIncentiveOutOfLane(const IdmPlannerParameters<T>& idm_params,
                                 const MobilPlannerParameters<T>& mobil_params, const ClosestPoses& closest_poses,
                                 const ClosestPose<T>& ego_closest_pose, const T& ego_old_accel,
                                 const T& trailing_delta_accel_this, T* incentive) const;

  // Computes an acceleration based on the IDM equation (via a call to
  // IdmPlanner::Eval()).
  const T EvaluateIdm(const IdmPlannerParameters<T>& idm_params, const ClosestPose<T>& trailing_closest_pose,
                      const ClosestPose<T>& leading_closest_pose) const;

  static constexpr int kIdmParamsIndex{0};
  static constexpr int kMobilParamsIndex{1};
  static constexpr double kDefaultLargeAccel{1e6};  // m/s^2

  const maliput::api::RoadGeometry& road_;
  const bool with_s_{true};
  const RoadPositionStrategy road_position_strategy_{};

  // Indices for the input / output ports.
  const int ego_pose_index_{};
  const int ego_velocity_index_{};
  const int ego_acceleration_index_{};
  const int traffic_index_{};
  const int lane_index_{};
};

}  // namespace delphyne
