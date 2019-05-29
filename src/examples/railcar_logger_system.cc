// Copyright 2019 Toyota Research Institute
#include "examples/railcar_logger_system.h"

#include <iostream>
#include <string>

#include "gen/rail_follower_state.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/value.h"

namespace delphyne {

RailcarLoggerSystem::RailcarLoggerSystem() :
    drake::systems::LeafSystem<double>(),
    ego_pose_index_(this->DeclareVectorInputPort(
        drake::systems::rendering::PoseVector<double>()).get_index()),
    ego_velocity_index_(this->DeclareVectorInputPort(
        drake::systems::rendering::FrameVelocity<double>()).get_index()),
    car_state_index_(this->DeclareVectorInputPort(
        delphyne::RailFollowerState<double>()).get_index()),
    lane_state_index_(this->DeclareAbstractInputPort(
        drake::Value<LaneDirection>()).get_index())
    {}

const drake::systems::InputPort<double>&
RailcarLoggerSystem::ego_pose_input() const {
  return drake::systems::System<double>::get_input_port(ego_pose_index_);
}

const drake::systems::InputPort<double>&
RailcarLoggerSystem::ego_velocity_input() const {
  return drake::systems::System<double>::get_input_port(ego_velocity_index_);
}

const drake::systems::InputPort<double>&
RailcarLoggerSystem::car_state_input() const {
  return drake::systems::System<double>::get_input_port(car_state_index_);
}

const drake::systems::InputPort<double>&
RailcarLoggerSystem::lane_state_input() const {
  return drake::systems::System<double>::get_input_port(lane_state_index_);
}

void RailcarLoggerSystem::DoCalcTimeDerivatives(
    const drake::systems::Context<double>& context,
    drake::systems::ContinuousState<double>*) const {
  std::cout << "[RailcarLoggerSystem::DoCalcTimeDerivatives]" << std::endl;
  // Logs all the railcar outputs
  log_ego_pose(
      this->template EvalVectorInput<drake::systems::rendering::PoseVector>(
          context, ego_pose_index_),
      context.get_time());

  log_ego_velocity(
      this->template EvalVectorInput<drake::systems::rendering::FrameVelocity>(
          context, ego_velocity_index_),
      context.get_time());

  log_car_state(
      this->template EvalVectorInput<delphyne::RailFollowerState>(
          context, car_state_index_),
      context.get_time());

  log_lane_state(
      this->template EvalInputValue<LaneDirection>(
          context, lane_state_index_),
      context.get_time());
}

void RailcarLoggerSystem::log_ego_pose(
    const drake::systems::rendering::PoseVector<double>* const ego_pose,
    const double& time) const {
  DRAKE_DEMAND(ego_pose != nullptr);
  std::cout << "\t[ego_pose]: ["
      << "( x: " << ego_pose->get_translation().x()
      << ", y: " << ego_pose->get_translation().y()
      << ", z: " << ego_pose->get_translation().z() << ")]"
      << ", t: " << time
      << std::endl;
}

void RailcarLoggerSystem::log_ego_velocity(
    const drake::systems::rendering::FrameVelocity<double>* const ego_velocity,
    const double& time) const {
  DRAKE_DEMAND(ego_velocity != nullptr);
  std::cout << "\t[ego_velocity]: ["
      << "( x: " << ego_velocity->get_velocity().translational().x()
      << ", y: " << ego_velocity->get_velocity().translational().y()
      << ", z: " << ego_velocity->get_velocity().translational().z() << ")]"
      << ", v: " << ego_velocity->get_velocity().translational().norm()
      << ", t: " << time
      << std::endl;
}

void RailcarLoggerSystem::log_car_state(
    const delphyne::RailFollowerState<double>* const car_state,
    const double& time) const {
  DRAKE_DEMAND(car_state != nullptr);
  std::cout << "\t[car_state]: ["
      << "( s: " << car_state->s()
      << ", v: " << car_state->speed() << ")]"
      << ", t: " << time
      << std::endl;
}

void RailcarLoggerSystem::log_lane_state(
    const LaneDirection* const lane_direction,
    const double& time) const {
  DRAKE_DEMAND(lane_direction != nullptr);
  std::cout << "\t[lane_state]: ["
      << "( lane_id: " << lane_direction->lane->id().string()
      << ", with_s: " << (lane_direction->with_s ? "true" : "false") << ")]"
      << ", t: " << time
      << std::endl;
}

}  // namespace delphyne

