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

#include "systems/idm_controller.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>
#include <drake/common/extract_double.h>
#include <drake/math/rigid_transform.h>

namespace delphyne {

using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseBundle;
using drake::systems::rendering::PoseVector;
using maliput::api::InertialPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;

static constexpr int kIdmParamsIndex{0};

template <typename T>
IDMController<T>::IDMController(const RoadGeometry& road, ScanStrategy path_or_branches,
                                RoadPositionStrategy road_position_strategy, double period_sec)
    : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<IDMController>{}),
      road_(road),
      path_or_branches_(path_or_branches),
      road_position_strategy_(road_position_strategy),
      period_sec_(period_sec),
      ego_pose_index_(this->DeclareVectorInputPort(PoseVector<T>()).get_index()),
      ego_velocity_index_(this->DeclareVectorInputPort(FrameVelocity<T>()).get_index()),
      traffic_index_(
          this->DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<PoseBundle<T>>()).get_index()),
      acceleration_index_(
          this->DeclareVectorOutputPort(drake::systems::BasicVector<T>(1), &IDMController::CalcAcceleration)
              .get_index()) {
  this->DeclareNumericParameter(IdmPlannerParameters<T>());
  // TODO(jadecastro) It is possible to replace the following AbstractState with
  // a caching sceme once #4364 lands, preventing the need to use abstract
  // states and periodic sampling time.
  if (road_position_strategy == RoadPositionStrategy::kCache) {
    this->DeclareAbstractState(drake::Value<RoadPosition>{});
    this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
  }
}

template <typename T>
IDMController<T>::~IDMController() {}

template <typename T>
const drake::systems::InputPort<T>& IDMController<T>::ego_pose_input() const {
  return drake::systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const drake::systems::InputPort<T>& IDMController<T>::ego_velocity_input() const {
  return drake::systems::System<T>::get_input_port(ego_velocity_index_);
}

template <typename T>
const drake::systems::InputPort<T>& IDMController<T>::traffic_input() const {
  return drake::systems::System<T>::get_input_port(traffic_index_);
}

template <typename T>
const drake::systems::OutputPort<T>& IDMController<T>::acceleration_output() const {
  return drake::systems::System<T>::get_output_port(acceleration_index_);
}

template <typename T>
void IDMController<T>::CalcAcceleration(const drake::systems::Context<T>& context,
                                        drake::systems::BasicVector<T>* accel_output) const {
  // Obtain the parameters.
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context, kIdmParamsIndex);
  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose = this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context, ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  const PoseBundle<T>* const traffic_poses = this->template EvalInputValue<PoseBundle<T>>(context, traffic_index_);
  DRAKE_ASSERT(traffic_poses != nullptr);

  // Obtain the state if we've allocated it.
  RoadPosition ego_rp;
  // if (context.template get_state().get_abstract_state().size() != 0) {
  if (context.get_state().get_abstract_state().size() != 0) {
    DRAKE_ASSERT(context.num_abstract_states() == 1);
    ego_rp = context.template get_abstract_state<RoadPosition>(0);
  }

  ImplCalcAcceleration(*ego_pose, *ego_velocity, *traffic_poses, idm_params, ego_rp, accel_output);
}

template <typename T>
void IDMController<T>::ImplCalcAcceleration(const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
                                            const PoseBundle<T>& traffic_poses,
                                            const IdmPlannerParameters<T>& idm_params, const RoadPosition& ego_rp,
                                            drake::systems::BasicVector<T>* command) const {
  using std::abs;
  using std::max;

  DRAKE_DEMAND(idm_params.IsValid());
  RoadPosition ego_position = ego_rp;
  if (!ego_rp.lane) {
    const auto gp =
        InertialPosition::FromXyz({drake::ExtractDoubleOrThrow(ego_pose.get_transform().translation().x()),
                                   drake::ExtractDoubleOrThrow(ego_pose.get_transform().translation().y()),
                                   drake::ExtractDoubleOrThrow(ego_pose.get_transform().translation().z())});
    ego_position = road_.ToRoadPosition(gp).road_position;
  }

  // Find the single closest car ahead.
  const ClosestPose<T> lead_car_pose = TrafficPoseSelector<T>::FindSingleClosestPose(
      ego_position.lane, ego_pose, traffic_poses, idm_params.scan_ahead_distance(), AheadOrBehind::kAhead,
      path_or_branches_);
  const T headway_distance = lead_car_pose.distance;

  const LanePosition lane_position(ego_position.pos.s(), ego_position.pos.r(), ego_position.pos.h());
  const T s_dot_ego = TrafficPoseSelector<T>::GetSigmaVelocity({ego_position.lane, lane_position, ego_velocity});
  const T s_dot_lead = (abs(lead_car_pose.odometry.pos.s()) == std::numeric_limits<T>::infinity())
                           ? T(0.)
                           : TrafficPoseSelector<T>::GetSigmaVelocity(lead_car_pose.odometry);

  // Saturate the net_distance at `idm_params.distance_lower_limit()` away from
  // the ego car to avoid near-singular solutions inherent to the IDM equation.
  const T actual_headway = headway_distance - idm_params.bloat_diameter();
  const T net_distance = max(actual_headway, idm_params.distance_lower_limit());
  const T closing_velocity = s_dot_ego - s_dot_lead;

  // Compute the acceleration command from the IDM equation.
  (*command)[0] = IdmPlanner<T>::Evaluate(idm_params, s_dot_ego, net_distance, closing_velocity);
}

template <typename T>
void IDMController<T>::DoCalcUnrestrictedUpdate(const drake::systems::Context<T>& context,
                                                const std::vector<const drake::systems::UnrestrictedUpdateEvent<T>*>&,
                                                drake::systems::State<T>* state) const {
  DRAKE_ASSERT(context.num_abstract_states() == 1);

  // Obtain the input and state data.
  const PoseVector<T>* const ego_pose = this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context, ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  RoadPosition& rp = state->template get_mutable_abstract_state<RoadPosition>(0);

  CalcOngoingRoadPosition(*ego_pose, *ego_velocity, road_, &rp);
}

}  // namespace delphyne

// These instantiations must match the API documentation in idm_controller.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(class ::delphyne::IDMController)
