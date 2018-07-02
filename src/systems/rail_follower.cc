// Copyright 2017 Toyota Research Institute

/*****************************************************************************
** Includes
*****************************************************************************/

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <drake/automotive/calc_smooth_acceleration.h>
#include <drake/automotive/maliput/api/branch_point.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/lane_data.h>
#include <drake/common/cond.h>
#include <drake/common/drake_assert.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/multibody_tree/math/spatial_velocity.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/value.h>
#include <drake/systems/framework/vector_base.h>

#include <Eigen/Geometry>

#include "systems/rail_follower_params.h"
#include "systems/rail_follower_state.h"
#include "systems/rail_follower_system.h"

/*****************************************************************************
** Names
*****************************************************************************/

using drake::maliput::api::GeoPosition;
using drake::maliput::api::IsoLaneVelocity;
using drake::maliput::api::LaneEnd;
using drake::maliput::api::LanePosition;
using drake::maliput::api::Rotation;
using drake::systems::AbstractValue;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::ContinuousState;
using drake::systems::InputPortDescriptor;
using drake::systems::OutputPort;
using drake::systems::VectorBase;
using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;
using drake::systems::Event;
using drake::systems::UnrestrictedUpdateEvent;

namespace delphyne {


/*****************************************************************************
** Local Helper Functions
*****************************************************************************/

namespace {

// Finds our continuous state in a context.
template <typename T>
const RailFollowerState<T>& get_state(
    const drake::systems::Context<T>& context) {
  const RailFollowerState<T>* const state =
      dynamic_cast<const RailFollowerState<T>*>(
          &context.get_continuous_state_vector());
  DRAKE_DEMAND(state != nullptr);
  return *state;
}

// Finds the lane direction state variable in a context.
template <typename T>
const LaneDirection& get_lane_direction(
    const drake::systems::Context<T>& context) {
  return context.template get_abstract_state<LaneDirection>(0);
}

}  // namespace


/*****************************************************************************
** RailFollowerState
*****************************************************************************/

template <typename T>
const std::vector<std::string> RailFollowerState<T>::kNames{
  "s",
  "speed"
};

template <typename T>
const std::vector<std::string> RailFollowerState<T>::kDocStrings{
  "The longitudinal position along the current rail (m).",
  "The speed of the vehicle in physical space (not nec. rail speed) (m/s)."
};

template <typename T>
const std::vector<double> RailFollowerState<T>::kDefaults{0.0, 0.0};

/*****************************************************************************
** RailFollowerParams
*****************************************************************************/

template <typename T>
const std::vector<std::string> RailFollowerParams<T>::kNames{
  "r",
  "h",
  "max_speed",
  "velocity_limit_kp"
};

template <typename T>
const std::vector<std::string> RailFollowerParams<T>::kDocStrings{
  "The vehicles position in the lane's r-axis (perpendicular axis) (m).",
  "The vehicles height about the lane's surface (m).",
  "The limit on the vehicle's forward speed, in meters per second; this element must be positive (m/s).",
  "The smoothing constant for min/max velocity limits; this element must be positive."
};

template <typename T>
const std::vector<double> RailFollowerParams<T>::kDefaults{0.0, 0.0, 45.0, 10.0};

/*****************************************************************************
** RailFollowerSystem
*****************************************************************************/

template <typename T>
constexpr double RailFollowerSystem<T>::kLaneEndEpsilon;
template <typename T>
constexpr double RailFollowerSystem<T>::kTimeEpsilon;

template <typename T>
RailFollowerSystem<T>::RailFollowerSystem(const LaneDirection& initial_lane_direction)
    : RailFollowerSystem(initial_lane_direction, RailFollowerState<T>(),
                     RailFollowerParams<T>()) {}

template <typename T>
RailFollowerSystem<T>::RailFollowerSystem(
    const LaneDirection& initial_lane_direction,
    const RailFollowerState<T>& initial_context_state,
    const RailFollowerParams<T>& initial_context_parameters)
    : initial_lane_direction_(initial_lane_direction) {
  command_input_port_index_ =
      this->DeclareInputPort(drake::systems::kVectorValued, 1).get_index();

  state_output_port_index_ =
      this->DeclareVectorOutputPort(&RailFollowerSystem::CalcStateOutput)
          .get_index();
  simple_car_state_output_port_index_ =
      this->DeclareVectorOutputPort(&RailFollowerSystem::CalcSimpleCarStateOutput)
          .get_index();
  lane_state_output_port_index_ =
      this->DeclareAbstractOutputPort(LaneDirection(initial_lane_direction),
                                      &RailFollowerSystem::CalcLaneOutput)
          .get_index();
  pose_output_port_index_ =
      this->DeclareVectorOutputPort(&RailFollowerSystem::CalcPose).get_index();
  velocity_output_port_index_ =
      this->DeclareVectorOutputPort(&RailFollowerSystem::CalcVelocity).get_index();
  this->DeclareAbstractState(
      AbstractValue::Make<LaneDirection>(initial_lane_direction));
  this->DeclareContinuousState(initial_context_state);
  this->DeclareNumericParameter(initial_context_parameters);
}

template <typename T>
const InputPortDescriptor<T>& RailFollowerSystem<T>::command_input() const {
  return this->get_input_port(command_input_port_index_);
}

template <typename T>
const OutputPort<T>& RailFollowerSystem<T>::state_output() const {
  return this->get_output_port(state_output_port_index_);
}

template <typename T>
const OutputPort<T>& RailFollowerSystem<T>::simple_car_state_output() const {
  return this->get_output_port(simple_car_state_output_port_index_);
}

template <typename T>
const OutputPort<T>& RailFollowerSystem<T>::lane_state_output() const {
  return this->get_output_port(lane_state_output_port_index_);
}

template <typename T>
const OutputPort<T>& RailFollowerSystem<T>::pose_output() const {
  return this->get_output_port(pose_output_port_index_);
}

template <typename T>
const OutputPort<T>& RailFollowerSystem<T>::velocity_output() const {
  return this->get_output_port(velocity_output_port_index_);
}

template <typename T>
RailFollowerParams<T>& RailFollowerSystem<T>::get_mutable_parameters(
    drake::systems::Context<T>* context) const {
  return this->template GetMutableNumericParameter<RailFollowerParams>(
      context, 0);
}

template <typename T>
void RailFollowerSystem<T>::CalcStateOutput(const Context<T>& context,
                                        RailFollowerState<T>* output) const {
  const RailFollowerState<T>& state = get_state(context);
  output->set_value(state.get_value());

  // Don't allow small negative speed to escape our state.
  DRAKE_ASSERT(state.speed() >= -1e-3);
  using std::max;
  output->set_speed(max(T(0), state.speed()));
}

template <typename T>
void RailFollowerSystem<T>::CalcSimpleCarStateOutput(
    const Context<T>& context, drake::automotive::SimpleCarState<T>* output) const {
  // Obtains car pose.
  auto pose = std::make_unique<drake::systems::rendering::PoseVector<T>>();
  CalcPose(context, pose.get());
  const Eigen::Translation<T, 3> pose_translation = pose->get_translation();
  const Eigen::Quaternion<T> pose_rotation = pose->get_rotation();
  // Translates pose from quaternion to euler.
  const Eigen::Vector3d euler_rotation =
      pose_rotation.toRotationMatrix().eulerAngles(0, 1, 2);

  // Obtains car velocity.
  auto velocity =
      std::make_unique<drake::systems::rendering::FrameVelocity<T>>();
  CalcVelocity(context, velocity.get());
  const drake::multibody::SpatialVelocity<T> spatial_velocity =
      velocity->get_velocity();
  const double velocity_norm =
      static_cast<double>(spatial_velocity.translational().norm());

  // Fills the SimpleCarState message.
  drake::automotive::SimpleCarState<T> state{};
  state.set_x(pose_translation.x());
  state.set_y(pose_translation.y());
  state.set_heading(euler_rotation(2));
  state.set_velocity(velocity_norm);

  output->set_value(state.get_value());

  // Don't allow small negative velocities to escape the state.
  output->set_velocity(std::max(T(0), state.velocity()));
}

template <typename T>
void RailFollowerSystem<T>::CalcLaneOutput(const Context<T>& context,
                                       LaneDirection* output) const {
  const LaneDirection& lane_direction = get_lane_direction(context);
  *output = lane_direction;
}

template <typename T>
T RailFollowerSystem<T>::CalcR(const RailFollowerParams<T>& params,
                           const LaneDirection& lane_direction) const {
  if (lane_direction.with_s == initial_lane_direction_.with_s) {
    return params.r();
  } else {
    return -params.r();
  }
}

template <typename T>
const RailFollowerParams<T>& RailFollowerSystem<T>::get_parameters(
    const drake::systems::Context<T>& context) const {
  return this->template GetNumericParameter<RailFollowerParams>(context, 0);
}

template <typename T>
void RailFollowerSystem<T>::CalcPose(const Context<T>& context,
                                 PoseVector<T>* pose) const {
  // Start with context archeology.
  const RailFollowerParams<T>& params = get_parameters(context);
  const RailFollowerState<T>& state = get_state(context);
  const LaneDirection& lane_direction = get_lane_direction(context);

  const LanePosition lane_position(state.s(), CalcR(params, lane_direction),
                                   params.h());
  const GeoPosition geo_position =
      lane_direction.lane->ToGeoPosition(lane_position);
  const Rotation rotation = lane_direction.lane->GetOrientation(lane_position);

  using std::atan2;
  using std::sin;
  using std::cos;

  // Adjust the rotation based on whether the vehicle is traveling with s or
  // against s.
  const Rotation adjusted_rotation =
      (lane_direction.with_s
           ? rotation
           : Rotation::FromRpy(
                 -rotation.roll(), -rotation.pitch(),
                 atan2(-sin(rotation.yaw()), -cos(rotation.yaw()))));
  pose->set_translation(Eigen::Translation<T, 3>(geo_position.xyz()));
  const drake::math::RollPitchYaw<T> rpy(adjusted_rotation.roll(),
                                  adjusted_rotation.pitch(),
                                  adjusted_rotation.yaw());
  pose->set_rotation(rpy.ToQuaternion());
}

template <typename T>
void RailFollowerSystem<T>::CalcVelocity(const Context<T>& context,
                                     FrameVelocity<T>* frame_velocity) const {
  // Start with context archeology.
  const RailFollowerParams<T>& params = get_parameters(context);
  const RailFollowerState<T>& state = get_state(context);
  const LaneDirection& lane_direction = get_lane_direction(context);

  // In the following code:
  //  - v is the translational component of the spatial velocity.
  //  - C is the car's frame.
  //  - L is the lane frame.
  //  - W is the world frame.
  //  - R is a rotation matrix.

  const drake::Vector3<T> v_LC_L(
      lane_direction.with_s ? state.speed() : -state.speed(), 0 /* r_dot */,
      0 /* h_dot */);
  const Rotation rotation = lane_direction.lane->GetOrientation(
      LanePosition(state.s(), params.r(), params.h()));
  const Eigen::Matrix<T, 3, 3> R_WL = rotation.matrix();
  const drake::Vector3<T> v_WC_W = R_WL * v_LC_L;

  // TODO(liang.fok) Add support for non-zero rotational velocity. See #5751.
  const drake::Vector3<T> w(T(0), T(0), T(0));
  frame_velocity->set_velocity(drake::multibody::SpatialVelocity<T>(w, v_WC_W));
}

template <typename T>
void RailFollowerSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT(derivatives != nullptr);

  const RailFollowerParams<T>& params = get_parameters(context);
  const RailFollowerState<T>& state = get_state(context);
  const LaneDirection& lane_direction = get_lane_direction(context);

  // Obtains the input.
  const BasicVector<T>* input = this->template EvalVectorInput<BasicVector>(
      context, command_input_port_index_);

  // Allocates and uses a BasicVector containing a zero acceleration command in
  // case the input contains nullptr.
  const auto default_input = BasicVector<T>::Make(0);
  if (input == nullptr) {
    input = default_input.get();
  }
  DRAKE_ASSERT(input->size() == 1);

  // Obtains the result structure.
  VectorBase<T>& vector_derivatives = derivatives->get_mutable_vector();
  RailFollowerState<T>* const rates =
      dynamic_cast<RailFollowerState<T>*>(&vector_derivatives);
  DRAKE_ASSERT(rates != nullptr);

  ImplCalcTimeDerivatives(params, state, lane_direction, *input, rates);
}

template <typename T>
void RailFollowerSystem<T>::ImplCalcTimeDerivatives(
    const RailFollowerParams<T>& params, const RailFollowerState<T>& state,
    const LaneDirection& lane_direction, const BasicVector<T>& input,
    RailFollowerState<T>* rates) const {
  const T speed = state.speed();
  const T sigma_v = drake::cond(lane_direction.with_s, speed, -speed);
  const LanePosition motion_derivatives =
      lane_direction.lane->EvalMotionDerivatives(
          LanePosition(state.s(), CalcR(params, lane_direction), params.h()),
          IsoLaneVelocity(sigma_v, 0 /* rho_v */, 0 /* eta_v */));
  // Since the railcar's IsoLaneVelocity's rho_v and eta_v values are both
  // zero, we expect the resulting motion derivative's r and h values to
  // also be zero. The IsoLaneVelocity's sigma_v, which may be non-zero, maps
  // to the motion derivative's s value.
  DRAKE_ASSERT(motion_derivatives.r() == 0);
  DRAKE_ASSERT(motion_derivatives.h() == 0);
  rates->set_s(motion_derivatives.s());

  const T desired_acceleration = input.GetAtIndex(0);
  const T smooth_acceleration =
      drake::automotive::calc_smooth_acceleration(desired_acceleration, params.max_speed(),
                               params.velocity_limit_kp(), state.speed());
  rates->set_speed(smooth_acceleration);
}

template <typename T>
drake::optional<bool> RailFollowerSystem<T>::DoHasDirectFeedthrough(int, int) const {
  return false;
}

// TODO(liang.fok): Switch to guard functions once they are available. The
// following computes an estimate of when the vehicle will reach the end of
// its lane. This estimate will be off when r != 0 and the lane is very
// curvy because the scale factors used in Lane::EvalMotionDerivatives() will
// not be constant.
//
// Another reason why the estimate will be off is the acceleration of the
// vehicle is not considered (see #5532).
template <typename T>
void RailFollowerSystem<T>::DoCalcNextUpdateTime(
    const drake::systems::Context<T>& context,
    drake::systems::CompositeEventCollection<T>* events, T* time) const {
  const RailFollowerState<T>& state = get_state(context);

  if (state.speed() == 0) {
    *time = T(std::numeric_limits<double>::infinity());
  } else {
    const RailFollowerParams<T>& params = get_parameters(context);
    const LaneDirection& lane_direction = get_lane_direction(context);

    const T& s = state.s();
    const T& speed = state.speed();
    const drake::maliput::api::Lane* lane = lane_direction.lane;
    const bool with_s = lane_direction.with_s;

    DRAKE_ASSERT(lane != nullptr);

    // Computes `s_dot`, the time derivative of `s`.
    const T sigma_v = drake::cond(with_s, speed, -speed);
    const LanePosition motion_derivatives =
        lane_direction.lane->EvalMotionDerivatives(
            LanePosition(s, CalcR(params, lane_direction), params.h()),
            IsoLaneVelocity(sigma_v, 0 /* rho_v */, 0 /* eta_v */));
    const T s_dot = motion_derivatives.s();

    const T distance = drake::cond(with_s, T(lane->length()) - s, -s);

    *time = context.get_time() + distance / s_dot;
  }

  // Gracefully handle the situation when the next update time is equal to the
  // current time. Since the integrator requires that the next update time be
  // strictly greater than the current time, a small time epsilon is used.
  if (*time == context.get_time()) {
    *time = context.get_time() + kTimeEpsilon;
  }
  events->add_unrestricted_update_event(
      std::make_unique<UnrestrictedUpdateEvent<T>>(
          Event<T>::TriggerType::kTimed));
}

template <typename T>
void RailFollowerSystem<T>::DoCalcUnrestrictedUpdate(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::UnrestrictedUpdateEvent<T>*>&,
    drake::systems::State<T>* next_state) const {
  const RailFollowerState<T>& current_railcar_state = get_state(context);
  const LaneDirection& current_lane_direction = get_lane_direction(context);
  DRAKE_ASSERT(current_lane_direction.lane != nullptr);
  const bool current_with_s = current_lane_direction.with_s;
  const double current_s = current_railcar_state.s();
  const double current_length = current_lane_direction.lane->length();

  // Copies the present state into the new one.
  next_state->CopyFrom(context.get_state());

  ContinuousState<T>& cs = next_state->get_mutable_continuous_state();
  VectorBase<T>& cv = cs.get_mutable_vector();
  RailFollowerState<T>* const next_railcar_state =
      dynamic_cast<RailFollowerState<T>*>(&cv);
  DRAKE_ASSERT(next_railcar_state != nullptr);

  // Handles the case where no lane change or speed adjustment is necessary. No
  // lane change is necessary when the vehicle is more than epilon away from the
  // next lane boundary.
  if ((current_with_s && current_s < current_length - kLaneEndEpsilon) ||
      (!current_with_s && current_s > kLaneEndEpsilon)) {
    return;
  }

  // Sets the speed to be zero if the car is at or is after the end of the road.
  if (current_with_s) {
    const int num_branches =
        current_lane_direction.lane->GetOngoingBranches(LaneEnd::kFinish)
            ->size();
    if (num_branches == 0 && current_s >= current_length - kLaneEndEpsilon) {
      next_railcar_state->set_speed(0);
    }
  } else {
    const int num_branches =
        current_lane_direction.lane->GetOngoingBranches(LaneEnd::kStart)
            ->size();
    if (num_branches == 0 && current_s <= kLaneEndEpsilon) {
      next_railcar_state->set_speed(0);
    }
  }

  if (next_railcar_state->speed() != 0) {
    LaneDirection& next_lane_direction =
        next_state->template get_mutable_abstract_state<LaneDirection>(0);
    // TODO(liang.fok) Generalize the following to support the selection of
    // non-default branches or non-zero ongoing branches. See #5702.
    drake::optional<LaneEnd> next_branch;
    if (current_with_s) {
      next_branch =
          current_lane_direction.lane->GetDefaultBranch(LaneEnd::kFinish);
      if (!next_branch) {
        const drake::maliput::api::LaneEndSet* ongoing_lanes =
            current_lane_direction.lane->GetOngoingBranches(LaneEnd::kFinish);
        if (ongoing_lanes != nullptr) {
          if (ongoing_lanes->size() > 0) {
            next_branch = ongoing_lanes->get(0);
          }
        }
      }
    } else {
      next_branch =
          current_lane_direction.lane->GetDefaultBranch(LaneEnd::kStart);
      if (!next_branch) {
        const drake::maliput::api::LaneEndSet* ongoing_lanes =
            current_lane_direction.lane->GetOngoingBranches(LaneEnd::kStart);
        if (ongoing_lanes != nullptr) {
          if (ongoing_lanes->size() > 0) {
            next_branch = ongoing_lanes->get(0);
          }
        }
      }
    }

    if (!next_branch) {
      DRAKE_ABORT_MSG(
          "RailFollower::DoCalcUnrestrictedUpdate: ERROR: "
          "Vehicle should switch lanes but no default or ongoing branch "
          "exists.");
    } else {
      next_lane_direction.lane = next_branch->lane;
      if (next_branch->end == LaneEnd::kStart) {
        next_lane_direction.with_s = true;
        next_railcar_state->set_s(0);
      } else {
        next_lane_direction.with_s = false;
        next_railcar_state->set_s(next_lane_direction.lane->length());
      }
    }
  }
}

// This section must match the API documentation in maliput_railcar.h.
template class RailFollowerSystem<double>;

}  // namespace delphyne
