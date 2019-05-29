// Copyright 2019 Toyota Research Institute
#include "examples/malidrive_automotive_simulator.h"

#include <algorithm>
#include <exception>

#include "gen/rail_follower_params.h"
#include "gen/rail_follower_state.h"

#include "drake/common/temp_directory.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/primitives/constant_value_source.h"


namespace delphyne {

MalidriveAutomotiveSimulator::MalidriveAutomotiveSimulator() {
  std::cout
      << "[MalidriveAutomotiveSimulator::MalidriveAutomotiveSimulator]"
      << std::endl;
  builder_ = std::make_unique<drake::systems::DiagramBuilder<double>>();
}

const maliput::api::RoadNetwork*
MalidriveAutomotiveSimulator::SetRoadNetwork(
    std::unique_ptr<const maliput::api::RoadNetwork> road_network) {
  std::cout << "[MalidriveAutomotiveSimulator::SetRoadNetwork]" << std::endl;
  DRAKE_DEMAND(!has_started());
  road_network_ = std::move(road_network);
  return road_network_.get();
}

int MalidriveAutomotiveSimulator::AddPriusMaliputRailcar(
    const std::string& name,
    const LaneDirection& initial_lane_direction,
    const delphyne::RailFollowerParams<double>& params,
    const delphyne::RailFollowerState<double>& initial_state,
    bool add_logger) {
  std::cout
      << "[MalidriveAutomotiveSimulator::AddPriusMaliputRailcar]"
      << std::endl;
  DRAKE_DEMAND(!has_started());
  CheckNameUniqueness(name);
  if (road_network_ == nullptr) {
    throw std::runtime_error(
      "MalidriveAutomotiveSimulator::AddPriusMaliputRailcar(): "
      "RoadNetwork not set. Please call SetRoadNetwork() first before "
      "calling this method.");
  }
  if (initial_lane_direction.lane == nullptr) {
    throw std::runtime_error(
      "MalidriveAutomotiveSimulator::AddPriusMaliputRailcar(): "
      "The provided initial lane is nullptr.");
  }
  if (initial_lane_direction.lane->segment()->junction()->road_geometry() !=
      road_network_->road_geometry()) {
    throw std::runtime_error(
      "MalidriveAutomotiveSimulator::AddPriusMaliputRailcar(): "
      "The provided initial lane is not within this simulation's "
      "RoadGeometry.");
  }

  const int id = allocate_vehicle_number();

  auto railcar =
      builder_->template AddSystem<delphyne::RailFollower<double>>(
          initial_lane_direction);
  railcar->set_name(name);
  vehicles_[id] = railcar;
  railcar_configs_[railcar].first.set_value(params.get_value());
  railcar_configs_[railcar].second.set_value(initial_state.get_value());

  if (add_logger) {
    railcar_loggers_[railcar] =
        builder_->template AddSystem<RailcarLoggerSystem>();

    builder_->Connect(railcar->pose_output(),
                      railcar_loggers_[railcar]->ego_pose_input());
    builder_->Connect(railcar->velocity_output(),
                      railcar_loggers_[railcar]->ego_velocity_input());
    builder_->Connect(railcar->state_output(),
                      railcar_loggers_[railcar]->car_state_input());
    builder_->Connect(railcar->lane_state_output(),
                      railcar_loggers_[railcar]->lane_state_input());
  }

  return id;
}

void MalidriveAutomotiveSimulator::Start(double target_realtime_rate) {
  std::cout << "[MalidriveAutomotiveSimulator::Start]" << std::endl;
  DRAKE_DEMAND(!has_started());

  BuildAndInitialize();

  simulator_->set_target_realtime_rate(target_realtime_rate);
  const double max_step_size = 0.01;
  simulator_->template reset_integrator<
      drake::systems::RungeKutta2Integrator<double>>(
          *diagram_, max_step_size, &simulator_->get_mutable_context());
  simulator_->get_mutable_integrator().set_fixed_step_mode(true);
  simulator_->Initialize();
}

void MalidriveAutomotiveSimulator::SetMaliputRailcarAccelerationCommand(int id,
    double acceleration) {
  std::cout <<
    "[MalidriveAutomotiveSimulator::SetMaliputRailcarAccelerationCommand]" <<
      std::endl;
  DRAKE_DEMAND(has_started());
  const auto iterator = vehicles_.find(id);
  if (iterator == vehicles_.end()) {
    throw std::runtime_error("MalidriveAutomotiveSimulator::"
        "SetMaliputRailcarAccelerationCommand(): Failed to find vehicle with "
        "id " + std::to_string(id) + ".");
  }
  RailFollower<double>* railcar =
      dynamic_cast<RailFollower<double>*>(
          iterator->second);
  if (railcar == nullptr) {
    throw std::runtime_error("MalidriveAutomotiveSimulator::"
        "SetMaliputRailcarAccelerationCommand(): The vehicle with "
        "id " + std::to_string(id) + " was not a RailFollower.");
  }
  DRAKE_ASSERT(diagram_ != nullptr);
  DRAKE_ASSERT(simulator_ != nullptr);
  drake::systems::Context<double>& context =
      diagram_->GetMutableSubsystemContext(
          *railcar, &simulator_->get_mutable_context());
  context.FixInputPort(railcar->command_input().get_index(),
      drake::systems::BasicVector<double>::Make(acceleration));
}

void MalidriveAutomotiveSimulator::StepBy(double time_step) {
  std::cout << "[MalidriveAutomotiveSimulator::StepBy]" << std::endl;
  const double time = simulator_->get_context().get_time();
  simulator_->StepTo(time + time_step);
}

void MalidriveAutomotiveSimulator::CheckNameUniqueness(
  const std::string& name) {
  std::cout <<
    "[MalidriveAutomotiveSimulator::CheckNameUniqueness]" << std::endl;
  for (const auto& vehicle : vehicles_) {
    if (vehicle.second->get_name() == name) {
      throw std::runtime_error("A vehicle named \"" + name + "\" already "
          "exists. It has id " + std::to_string(vehicle.first) + ".");
    }
  }
}

int MalidriveAutomotiveSimulator::allocate_vehicle_number() {
  std::cout <<
    "[MalidriveAutomotiveSimulator::allocate_vehicle_number]" << std::endl;
  DRAKE_DEMAND(!has_started());
  return next_vehicle_number_++;
}

void MalidriveAutomotiveSimulator::BuildAndInitialize() {
  std::cout <<
    "[MalidriveAutomotiveSimulator::BuildAndInitialize]" << std::endl;
  DRAKE_DEMAND(!has_started());

  if (diagram_ == nullptr) {
    Build();
  }
  simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_);
  InitializeMaliputRailcars();
}

void MalidriveAutomotiveSimulator::Build() {
  std::cout << "[MalidriveAutomotiveSimulator::Build]" << std::endl;
  DRAKE_DEMAND(diagram_ == nullptr);
  diagram_ = builder_->Build();
  diagram_->set_name("MalidriveAutomotiveSimulator");
}

void MalidriveAutomotiveSimulator::InitializeMaliputRailcars() {
  std::cout <<
    "[MalidriveAutomotiveSimulator::InitializeMaliputRailcars]" << std::endl;
  for (auto& pair : railcar_configs_) {
    const RailFollower<double>* const car = pair.first;
    const RailFollowerParams<double>& params = pair.second.first;
    const RailFollowerState<double>& initial_state = pair.second.second;

    drake::systems::Context<double>& context =
        diagram_->GetMutableSubsystemContext(
            *car, &simulator_->get_mutable_context());

    drake::systems::VectorBase<double>& context_state =
        context.get_mutable_continuous_state_vector();
    RailFollowerState<double>* const state =
        dynamic_cast<RailFollowerState<double>*>(&context_state);
    DRAKE_ASSERT(state);
    state->set_value(initial_state.get_value());

    RailFollowerParams<double>& railcar_system_params =
        car->get_mutable_parameters(&context);
    railcar_system_params.set_value(params.get_value());
  }
}

}  // namespace delphyne
