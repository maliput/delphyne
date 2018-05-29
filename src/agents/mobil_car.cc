/**
 * @file src/agents/mobil_car.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/mobil_car.h"

#include <memory>
#include <string>

#include "drake/automotive/idm_controller.h"
#include "drake/automotive/mobil_planner.h"
#include "drake/automotive/prius_vis.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/systems/primitives/multiplexer.h"

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include "backend/ign_publisher_system.h"
#include "backend/translation_systems/drake_simple_car_state_to_ign.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

MobilCar::MobilCar(const std::string& name, const bool& direction_of_travel,
                   const double& x, const double& y, const double& heading,
                   const double& speed)
    : delphyne::Agent(name),
      initial_parameters_(direction_of_travel, x, y, heading, speed),
      simple_car_system_(nullptr) {
  igndbg << "MobilCar constructor" << std::endl;
}

int MobilCar::Configure(
    const int& id,
    const std::unique_ptr<const drake::maliput::api::RoadGeometry>&
        road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "MobilCar configure" << std::endl;

  /*********************
   * Basics
   *********************/
  id_ = id;

  /*********************
   * Checks
   *********************/
  if (!road_geometry) {
    ignerr << "Rail cars need a road geometry to drive on, make sure "
           << "the simulation is configured with one." << std::endl;
    return -1;
  }

  /*********************
   * Systems
   *********************/
  // driving
  drake::automotive::MobilPlanner<double>* mobil_planner =
      builder->template AddSystem<drake::automotive::MobilPlanner<double>>(
          std::make_unique<drake::automotive::MobilPlanner<double>>(
              *road_geometry, initial_parameters_.direction_of_travel,
              drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
              0. /* time period (unused) */));
  mobil_planner->set_name(name_ + "_mobil_planner");

  drake::automotive::IdmController<double>* idm_controller =
      builder->template AddSystem<drake::automotive::IdmController<double>>(
          std::make_unique<drake::automotive::IdmController<double>>(
              *road_geometry, drake::automotive::ScanStrategy::kBranches,
              drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
              0. /* time period (unused) */));
  idm_controller->set_name(name_ + "_idm_controller");

  drake::automotive::PurePursuitController<double>* pursuit =
      builder->template AddSystem<
          drake::automotive::PurePursuitController<double>>(
          std::make_unique<drake::automotive::PurePursuitController<double>>());
  pursuit->set_name(name_ + "_pure_pursuit_controller");

  simple_car_system_ =
      builder->template AddSystem<drake::automotive::SimpleCar2<double>>(
          std::make_unique<drake::automotive::SimpleCar2<double>>());
  simple_car_system_->set_name(name_ + "_simple_car");

  drake::systems::Multiplexer<double>* mux =
      builder->template AddSystem<drake::systems::Multiplexer<double>>(
          std::make_unique<drake::systems::Multiplexer<double>>(
              drake::automotive::DrivingCommand<double>()));
  mux->set_name(name_ + "_mux");

  // publishing
  auto car_state_translator =
      builder->template AddSystem<DrakeSimpleCarStateToIgn>();

  const std::string car_state_channel =
      "agents/" + std::to_string(id) + "/state";

  IgnPublisherSystem<ignition::msgs::SimpleCarState>* car_state_publisher =
      builder->template AddSystem<
          IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
          std::make_unique<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
              car_state_channel));

  /*********************
   * Diagram Wiring
   *********************/
  // driving
  builder->Connect(simple_car_system_->pose_output(),
                   mobil_planner->ego_pose_input());
  builder->Connect(simple_car_system_->velocity_output(),
                   mobil_planner->ego_velocity_input());
  builder->Connect(idm_controller->acceleration_output(),
                   mobil_planner->ego_acceleration_input());
  builder->Connect(aggregator->get_output_port(0),
                   mobil_planner->traffic_input());

  builder->Connect(simple_car_system_->pose_output(),
                   idm_controller->ego_pose_input());
  builder->Connect(simple_car_system_->velocity_output(),
                   idm_controller->ego_velocity_input());
  builder->Connect(aggregator->get_output_port(0),
                   idm_controller->traffic_input());

  builder->Connect(simple_car_system_->pose_output(),
                   pursuit->ego_pose_input());
  builder->Connect(mobil_planner->lane_output(), pursuit->lane_input());
  // Build DrivingCommand via a mux of two scalar outputs (a BasicVector where
  // row 0 = steering command, row 1 = acceleration command).
  builder->Connect(pursuit->steering_command_output(), mux->get_input_port(0));
  builder->Connect(idm_controller->acceleration_output(),
                   mux->get_input_port(1));
  builder->Connect(mux->get_output_port(0),
                   simple_car_system_->get_input_port(0));

  // simulator
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  auto ports = aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(simple_car_system_->pose_output(), ports.pose_descriptor);
  builder->Connect(simple_car_system_->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id, name_));

  // publishing
  builder->Connect(simple_car_system_->state_output(),
                   car_state_translator->get_input_port(0));
  builder->Connect(*car_state_translator, *car_state_publisher);

  return 0;
}

int MobilCar::Initialize(drake::systems::Context<double>* system_context) {
  igndbg << "MobilCar initialize" << std::endl;

  typedef drake::automotive::SimpleCarState<double> SimpleCarContextState;
  typedef std::unique_ptr<SimpleCarContextState> SimpleCarContextStatePtr;

  auto simple_car_state = std::make_unique<SimpleCarContextState>();
  // TODO(daniel.stonier) check for 'in-lane' bounds?
  simple_car_state->set_x(initial_parameters_.x);
  simple_car_state->set_y(initial_parameters_.y);
  simple_car_state->set_heading(initial_parameters_.heading);
  simple_car_state->set_velocity(initial_parameters_.speed);

  drake::systems::VectorBase<double>& context_state =
      system_context->get_mutable_continuous_state().get_mutable_vector();
  drake::systems::BasicVector<double>* const state =
      dynamic_cast<drake::systems::BasicVector<double>*>(&context_state);
  DELPHYNE_ASSERT(state);
  state->set_value(simple_car_state->get_value());

  return 0;
}

drake::systems::System<double>* MobilCar::get_system() const {
  return simple_car_system_;
}

}  // namespace delphyne
