// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.

// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:

// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// clalancette, caguero: The vast majority of the code below is copied from
// https://github.com/RobotLocomotion/drake/blob/f6f23c5bc539a6aaf754c27b69ef14a69ab3430f/automotive/simple_car.cc
// and
// https://github.com/RobotLocomotion/drake/blob/f6f23c5bc539a6aaf754c27b69ef14a69ab3430f/automotive/simple_car.h
// and
// https://github.com/RobotLocomotion/drake/blob/f6f23c5bc539a6aaf754c27b69ef14a69ab3430f/automotive/automotive_simulator.cc
//
// I've modified it slightly to
// 1) Combine the .cc and .h into one file (given that we build this into a .so
//    there is no reason to have the separated file),
// 2) Add in namespaces, and
// 3) Add in the pieces necessary to make it a loadable agent (registering it
//    as a shared library and overriding the appropriate methods).

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <experimental/optional>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/cond.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw_using_quaternion.h"
#include "drake/math/saturate.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

#include <backend/agent_plugin_base.h>
#include <backend/linb-any>

namespace delphyne {
namespace backend {

namespace {

// Finds our continuous state in a context.
template <typename T>
const drake::automotive::MaliputRailcarState<T>& get_state(
    const drake::systems::Context<T>& context) {
  const drake::automotive::MaliputRailcarState<T>* const state =
      dynamic_cast<const drake::automotive::MaliputRailcarState<T>*>(
          &context.get_continuous_state_vector());
  DRAKE_DEMAND(state != nullptr);
  return *state;
}

// Finds the lane direction state variable in a context.
template <typename T>
const drake::automotive::LaneDirection& get_lane_direction(
    const drake::systems::Context<T>& context) {
  return context.template get_abstract_state<drake::automotive::LaneDirection>(
      0);
}

}  // namespace

class LoadableMaliputRailcarDouble final
    : public delphyne::backend::AgentPluginDoubleBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LoadableMaliputRailcarDouble)

  static constexpr double kLaneEndEpsilon{1e-12};
  static constexpr double kTimeEpsilon{1e-12};
  static constexpr double kDefaultInitialS{0};
  static constexpr double kDefaultInitialSpeed{1};

  LoadableMaliputRailcarDouble() {
    igndbg << "LoadableMaliputRailcar constructor" << std::endl;
    command_input_port_index_ =
        this->DeclareInputPort(drake::systems::kVectorValued, 1).get_index();

    state_output_port_index_ =
        this->DeclareVectorOutputPort(
                &LoadableMaliputRailcarDouble::CalcStateOutput)
            .get_index();
    pose_output_port_index_ =
        this->DeclareVectorOutputPort(&LoadableMaliputRailcarDouble::CalcPose)
            .get_index();
    velocity_output_port_index_ =
        this->DeclareVectorOutputPort(
                &LoadableMaliputRailcarDouble::CalcVelocity)
            .get_index();

    this->DeclareContinuousState(
        drake::automotive::MaliputRailcarState<double>());
    this->DeclareNumericParameter(
        drake::automotive::MaliputRailcarParams<double>());
  }

  int Configure(const std::map<std::string, linb::any>& parameters,
                drake::systems::DiagramBuilder<double>* builder,
                drake::lcm::DrakeLcmInterface* lcm, const std::string& name,
                int id,
                drake::systems::rendering::PoseAggregator<double>* aggregator,
                drake::automotive::CarVisApplicator<double>* car_vis_applicator)
      override {
    igndbg << "LoadableMaliputRailcar configure" << std::endl;
    initial_lane_direction_ = linb::any_cast<drake::automotive::LaneDirection*>(
        parameters.at("lane_direction"));

    auto road = linb::any_cast<const drake::maliput::api::RoadGeometry*>(
        parameters.at("road"));

    params_ = linb::any_cast<drake::automotive::MaliputRailcarParams<double>*>(
        parameters.at("start_params"));

    if (road == nullptr) {
      ignerr << "AutomotiveSimulator::AddPriusMaliputRailcar(): "
                "RoadGeometry not set. Please call SetRoadGeometry() first "
                "before calling this method."
             << std::endl;
      return -1;
    }
    if (initial_lane_direction_->lane == nullptr) {
      ignerr << "AutomotiveSimulator::AddPriusMaliputRailcar(): "
                "The provided initial lane is nullptr."
             << std::endl;
      return -1;
    }
    if (initial_lane_direction_->lane->segment()->junction()->road_geometry() !=
        road) {
      ignerr << "AutomotiveSimulator::AddPriusMaliputRailcar(): "
                "The provided initial lane is not within this simulation's "
                "RoadGeometry."
             << std::endl;
      return -1;
    }

    lane_state_output_port_index_ =
        this->DeclareAbstractOutputPort(
                *initial_lane_direction_,
                &LoadableMaliputRailcarDouble::CalcLaneOutput)
            .get_index();

    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
    builder->Connect(this->pose_output(), ports.pose_descriptor);
    builder->Connect(this->velocity_output(), ports.velocity_descriptor);
    car_vis_applicator->AddCarVis(
        std::make_unique<drake::automotive::PriusVis<double>>(id, name));

    return 0;
  }

  int Initialize(drake::systems::Context<double>* context) override {
    igndbg << "LoadableMaliputRailcar initialize" << std::endl;
    drake::automotive::MaliputRailcarParams<double>& railcar_system_params =
        this->get_mutable_parameters(context);
    railcar_system_params.set_value(params_->get_value());

    return 0;
  }

 private:
  const drake::systems::InputPortDescriptor<double>& command_input() const {
    return this->get_input_port(command_input_port_index_);
  }

  const drake::systems::OutputPort<double>& state_output() const {
    return this->get_output_port(state_output_port_index_);
  }

  const drake::systems::OutputPort<double>& lane_state_output() const {
    return this->get_output_port(lane_state_output_port_index_);
  }

  const drake::systems::OutputPort<double>& pose_output() const {
    return this->get_output_port(pose_output_port_index_);
  }

  const drake::systems::OutputPort<double>& velocity_output() const {
    return this->get_output_port(velocity_output_port_index_);
  }

  drake::automotive::MaliputRailcarParams<double>& get_mutable_parameters(
      drake::systems::Context<double>* context) const {
    return this->template GetMutableNumericParameter<
        drake::automotive::MaliputRailcarParams>(context, 0);
  }

  void CalcStateOutput(
      const drake::systems::Context<double>& context,
      drake::automotive::MaliputRailcarState<double>* output) const {
    const drake::automotive::MaliputRailcarState<double>& state =
        get_state(context);
    output->set_value(state.get_value());

    // Don't allow small negative speed to escape our state.
    DRAKE_ASSERT(state.speed() >= -1e-3);
    using std::max;
    output->set_speed(max(0.0, state.speed()));
  }

  void CalcLaneOutput(const drake::systems::Context<double>& context,
                      drake::automotive::LaneDirection* output) const {
    const drake::automotive::LaneDirection& lane_direction =
        get_lane_direction(context);
    *output = lane_direction;
  }

  double CalcR(const drake::automotive::MaliputRailcarParams<double>& params,
               const drake::automotive::LaneDirection& lane_direction) const {
    if (lane_direction.with_s == initial_lane_direction_->with_s) {
      return params.r();
    } else {
      return -params.r();
    }
  }

  const drake::automotive::MaliputRailcarParams<double>& get_parameters(
      const drake::systems::Context<double>& context) const {
    return this
        ->template GetNumericParameter<drake::automotive::MaliputRailcarParams>(
            context, 0);
  }

  void CalcPose(const drake::systems::Context<double>& context,
                drake::systems::rendering::PoseVector<double>* pose) const {
    // Start with context archeology.
    const drake::automotive::MaliputRailcarParams<double>& params =
        get_parameters(context);
    const drake::automotive::MaliputRailcarState<double>& state =
        get_state(context);

    const drake::automotive::LaneDirection& lane_direction =
        get_lane_direction(context);
    const drake::maliput::api::LanePosition lane_position(
        state.s(), CalcR(params, lane_direction), params.h());

    const drake::maliput::api::GeoPosition geo_position =
        lane_direction.lane->ToGeoPosition(lane_position);

    const drake::maliput::api::Rotation rotation =
        lane_direction.lane->GetOrientation(lane_position);

    using std::atan2;
    using std::sin;
    using std::cos;

    // Adjust the rotation based on whether the vehicle is traveling with s or
    // against s.
    const drake::maliput::api::Rotation adjusted_rotation =
        (lane_direction.with_s
             ? rotation
             : drake::maliput::api::Rotation::FromRpy(
                   -rotation.roll(), -rotation.pitch(),
                   atan2(-sin(rotation.yaw()), -cos(rotation.yaw()))));
    pose->set_translation(Eigen::Translation<double, 3>(geo_position.xyz()));
    pose->set_rotation(
        drake::math::RollPitchYawToQuaternion(drake::Vector3<double>(
            adjusted_rotation.roll(), adjusted_rotation.pitch(),
            adjusted_rotation.yaw())));
  }

  void CalcVelocity(
      const drake::systems::Context<double>& context,
      drake::systems::rendering::FrameVelocity<double>* frame_velocity) const {
    // Start with context archeology.
    const drake::automotive::MaliputRailcarParams<double>& params =
        get_parameters(context);
    const drake::automotive::MaliputRailcarState<double>& state =
        get_state(context);
    const drake::automotive::LaneDirection& lane_direction =
        get_lane_direction(context);

    // In the following code:
    //  - v is the translational component of the spatial velocity.
    //  - C is the car's frame.
    //  - L is the lane frame.
    //  - W is the world frame.
    //  - R is a rotation matrix.

    const drake::Vector3<double> v_LC_L(
        lane_direction.with_s ? state.speed() : -state.speed(), 0 /* r_dot */,
        0 /* h_dot */);
    const drake::maliput::api::Rotation rotation =
        lane_direction.lane->GetOrientation(drake::maliput::api::LanePosition(
            state.s(), params.r(), params.h()));
    const Eigen::Matrix<double, 3, 3> R_WL = rotation.matrix();
    const drake::Vector3<double> v_WC_W = R_WL * v_LC_L;

    // TODO(liang.fok) Add support for non-zero rotational velocity. See #5751.
    const drake::Vector3<double> w(0, 0, 0);
    frame_velocity->set_velocity(
        drake::multibody::SpatialVelocity<double>(w, v_WC_W));
  }

  void ImplCalcTimeDerivatives(
      const drake::automotive::MaliputRailcarParams<double>& params,
      const drake::automotive::MaliputRailcarState<double>& state,
      const drake::automotive::LaneDirection& lane_direction,
      const drake::systems::BasicVector<double>& input,
      drake::automotive::MaliputRailcarState<double>* rates) const {
    const double speed = state.speed();
    const double sigma_v = drake::cond(lane_direction.with_s, speed, -speed);
    const drake::maliput::api::LanePosition motion_derivatives =
        lane_direction.lane->EvalMotionDerivatives(
            drake::maliput::api::LanePosition(
                state.s(), CalcR(params, lane_direction), params.h()),
            drake::maliput::api::IsoLaneVelocity(sigma_v, 0 /* rho_v */,
                                                 0 /* eta_v */));
    // Since the railcar's IsoLaneVelocity's rho_v and eta_v values are both
    // zero, we expect the resulting motion derivative's r and h values to
    // also be zero. The IsoLaneVelocity's sigma_v, which may be non-zero, maps
    // to the motion derivative's s value.
    DRAKE_ASSERT(motion_derivatives.r() == 0);
    DRAKE_ASSERT(motion_derivatives.h() == 0);
    rates->set_s(motion_derivatives.s());

    const double desired_acceleration = input.GetAtIndex(0);
    const double smooth_acceleration =
        drake::automotive::calc_smooth_acceleration(
            desired_acceleration, params.max_speed(),
            params.velocity_limit_kp(), state.speed());
    rates->set_speed(smooth_acceleration);
  }

  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const {
    DRAKE_ASSERT(derivatives != nullptr);

    const drake::automotive::MaliputRailcarParams<double>& params =
        get_parameters(context);
    const drake::automotive::MaliputRailcarState<double>& state =
        get_state(context);
    const drake::automotive::LaneDirection& lane_direction =
        get_lane_direction(context);

    // Obtains the input.
    const drake::systems::BasicVector<double>* input =
        this->template EvalVectorInput<drake::systems::BasicVector>(
            context, command_input_port_index_);

    // Allocates and uses a BasicVector containing a zero acceleration command
    // in case the input contains nullptr.
    const auto default_input = drake::systems::BasicVector<double>::Make(0);
    if (input == nullptr) {
      input = default_input.get();
    }
    DRAKE_ASSERT(input->size() == 1);

    // Obtains the result structure.
    drake::systems::VectorBase<double>& vector_derivatives =
        derivatives->get_mutable_vector();
    drake::automotive::MaliputRailcarState<double>* const rates =
        dynamic_cast<drake::automotive::MaliputRailcarState<double>*>(
            &vector_derivatives);
    DRAKE_ASSERT(rates != nullptr);

    ImplCalcTimeDerivatives(params, state, lane_direction, *input, rates);
  }

  std::unique_ptr<drake::systems::AbstractValues> AllocateAbstractState()
      const {
    std::vector<std::unique_ptr<drake::systems::AbstractValue>> abstract_values;
    const drake::automotive::LaneDirection lane_direction;
    abstract_values.push_back(std::unique_ptr<drake::systems::AbstractValue>(
        std::make_unique<
            drake::systems::Value<drake::automotive::LaneDirection>>(
            lane_direction)));
    return std::make_unique<drake::systems::AbstractValues>(
        std::move(abstract_values));
  }

  drake::optional<bool> DoHasDirectFeedthrough(int, int) const { return false; }

  void SetDefaultState(
      drake::automotive::MaliputRailcarState<double>* railcar_state) const {
    railcar_state->set_s(kDefaultInitialS);
    railcar_state->set_speed(kDefaultInitialSpeed);
  }

  void SetDefaultState(const drake::systems::Context<double>&,
                       drake::systems::State<double>* state) const override {
    drake::automotive::MaliputRailcarState<double>* railcar_state =
        dynamic_cast<drake::automotive::MaliputRailcarState<double>*>(
            &state->get_mutable_continuous_state().get_mutable_vector());
    DRAKE_DEMAND(railcar_state != nullptr);
    SetDefaultState(railcar_state);

    drake::automotive::LaneDirection& lane_direction =
        state->get_mutable_abstract_state()
            .get_mutable_value(0)
            .template GetMutableValue<drake::automotive::LaneDirection>();
    lane_direction = *initial_lane_direction_;
  }

  // TODO(liang.fok): Switch to guard functions once they are available. The
  // following computes an estimate of when the vehicle will reach the end of
  // its lane. This estimate will be off when r != 0 and the lane is very
  // curvy because the scale factors used in Lane::EvalMotionDerivatives() will
  // not be constant.
  //
  // Another reason why the estimate will be off is the acceleration of the
  // vehicle is not considered (see #5532).
  void DoCalcNextUpdateTime(
      const drake::systems::Context<double>& context,
      drake::systems::CompositeEventCollection<double>* events,
      double* time) const {
    const drake::automotive::MaliputRailcarState<double>& state =
        get_state(context);

    if (state.speed() == 0) {
      *time = std::numeric_limits<double>::infinity();
    } else {
      const drake::automotive::MaliputRailcarParams<double>& params =
          get_parameters(context);
      const drake::automotive::LaneDirection& lane_direction =
          get_lane_direction(context);

      const double& s = state.s();
      const double& speed = state.speed();
      const drake::maliput::api::Lane* lane = lane_direction.lane;
      const bool with_s = lane_direction.with_s;

      DRAKE_ASSERT(lane != nullptr);

      // Computes `s_dot`, the time derivative of `s`.
      const double sigma_v = drake::cond(with_s, speed, -speed);
      const drake::maliput::api::LanePosition motion_derivatives =
          lane_direction.lane->EvalMotionDerivatives(
              drake::maliput::api::LanePosition(
                  s, CalcR(params, lane_direction), params.h()),
              drake::maliput::api::IsoLaneVelocity(sigma_v, 0 /* rho_v */,
                                                   0 /* eta_v */));
      const double s_dot = motion_derivatives.s();

      const double distance = drake::cond(with_s, lane->length() - s, -s);

      *time = context.get_time() + distance / s_dot;
    }

    // Gracefully handle the situation when the next update time is equal to the
    // current time. Since the integrator requires that the next update time be
    // strictly greater than the current time, a small time epsilon is used.
    if (*time == context.get_time()) {
      *time = context.get_time() + kTimeEpsilon;
    }
    events->add_unrestricted_update_event(
        std::make_unique<drake::systems::UnrestrictedUpdateEvent<double>>(
            drake::systems::Event<double>::TriggerType::kTimed));
  }

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      const std::vector<
          const drake::systems::UnrestrictedUpdateEvent<double>*>&,
      drake::systems::State<double>* next_state) const {
    const drake::automotive::MaliputRailcarState<double>&
        current_railcar_state = get_state(context);
    const drake::automotive::LaneDirection& current_lane_direction =
        get_lane_direction(context);
    DRAKE_ASSERT(current_lane_direction.lane != nullptr);
    const bool current_with_s = current_lane_direction.with_s;
    const double current_s = current_railcar_state.s();
    const double current_length = current_lane_direction.lane->length();

    // Copies the present state into the new one.
    next_state->CopyFrom(context.get_state());

    drake::systems::ContinuousState<double>& cs =
        next_state->get_mutable_continuous_state();
    drake::systems::VectorBase<double>& cv = cs.get_mutable_vector();
    drake::automotive::MaliputRailcarState<double>* const next_railcar_state =
        dynamic_cast<drake::automotive::MaliputRailcarState<double>*>(&cv);
    DRAKE_ASSERT(next_railcar_state != nullptr);

    // Handles the case where no lane change or speed adjustment is necessary.
    // No lane change is necessary when the vehicle is more than epilon away
    // from the next lane boundary.
    if ((current_with_s && current_s < current_length - kLaneEndEpsilon) ||
        (!current_with_s && current_s > kLaneEndEpsilon)) {
      return;
    }

    // Sets the speed to be zero if the car is at or is after the end
    // of the road.
    if (current_with_s) {
      const int num_branches =
          current_lane_direction.lane
              ->GetOngoingBranches(drake::maliput::api::LaneEnd::kFinish)
              ->size();
      if (num_branches == 0 && current_s >= current_length - kLaneEndEpsilon) {
        next_railcar_state->set_speed(0);
      }
    } else {
      const int num_branches =
          current_lane_direction.lane
              ->GetOngoingBranches(drake::maliput::api::LaneEnd::kStart)
              ->size();
      if (num_branches == 0 && current_s <= kLaneEndEpsilon) {
        next_railcar_state->set_speed(0);
      }
    }

    if (next_railcar_state->speed() != 0) {
      drake::automotive::LaneDirection& next_lane_direction =
          next_state->template get_mutable_abstract_state<
              drake::automotive::LaneDirection>(0);
      // TODO(liang.fok) Generalize the following to support the selection of
      // non-default branches or non-zero ongoing branches. See #5702.
      drake::optional<drake::maliput::api::LaneEnd> next_branch;
      if (current_with_s) {
        next_branch = current_lane_direction.lane->GetDefaultBranch(
            drake::maliput::api::LaneEnd::kFinish);
        if (!next_branch) {
          const drake::maliput::api::LaneEndSet* ongoing_lanes =
              current_lane_direction.lane->GetOngoingBranches(
                  drake::maliput::api::LaneEnd::kFinish);
          if (ongoing_lanes != nullptr) {
            if (ongoing_lanes->size() > 0) {
              next_branch = ongoing_lanes->get(0);
            }
          }
        }
      } else {
        next_branch = current_lane_direction.lane->GetDefaultBranch(
            drake::maliput::api::LaneEnd::kStart);
        if (!next_branch) {
          const drake::maliput::api::LaneEndSet* ongoing_lanes =
              current_lane_direction.lane->GetOngoingBranches(
                  drake::maliput::api::LaneEnd::kStart);
          if (ongoing_lanes != nullptr) {
            if (ongoing_lanes->size() > 0) {
              next_branch = ongoing_lanes->get(0);
            }
          }
        }
      }

      if (!next_branch) {
        DRAKE_ABORT_MSG(
            "MaliputRailcar::DoCalcUnrestrictedUpdate: ERROR: "
            "Vehicle should switch lanes but no default or ongoing "
            "branch exists.");
      } else {
        next_lane_direction.lane = next_branch->lane;
        if (next_branch->end == drake::maliput::api::LaneEnd::kStart) {
          next_lane_direction.with_s = true;
          next_railcar_state->set_s(0);
        } else {
          next_lane_direction.with_s = false;
          next_railcar_state->set_s(next_lane_direction.lane->length());
        }
      }
    }
  }

  drake::automotive::LaneDirection* initial_lane_direction_;
  int command_input_port_index_{};
  int state_output_port_index_{};
  int lane_state_output_port_index_{};
  int pose_output_port_index_{};
  int velocity_output_port_index_{};

  drake::automotive::MaliputRailcarParams<double>* params_;
};

class LoadableMaliputRailcarFactoryDouble final
    : public delphyne::backend::AgentPluginFactoryDoubleBase {
 public:
  std::unique_ptr<delphyne::backend::AgentPluginBase<double>> Create() {
    return std::make_unique<LoadableMaliputRailcarDouble>();
  }
};

}  // namespace backend
}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    delphyne::backend::LoadableMaliputRailcarFactoryDouble,
    delphyne::backend::AgentPluginFactoryDoubleBase)
