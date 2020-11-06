/**
 * @file src/agents/simple_car
 *
 * Copyright 2020 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/unicycle_car.h"

#include <string>
#include <utility>

#include "backend/ign_subscriber_system.h"
#include "gen/simple_car_state.h"
#include "systems/unicycle_car.h"
#include "translations/ign_angular_rate_acceleration_command_to_drake.h"

#include <maliput/common/maliput_unused.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

UnicycleCarBlueprint::UnicycleCarBlueprint(const std::string& name, double x, double y, double heading, double speed)
    : BasicAgentBlueprint(name), initial_conditions_(x, y, heading, speed) {}

std::unique_ptr<Agent::Diagram> UnicycleCarBlueprint::DoBuildDiagram(
    const maliput::api::RoadGeometry* road_geometry) const {
  maliput::common::unused(road_geometry);

  AgentBlueprint::DiagramBuilder builder(this->name());

  /*********************
   * Context
   *********************/
  typedef SimpleCarState<double> ContextContinuousState;
  ContextContinuousState context_continuous_state;
  context_continuous_state.set_x(initial_conditions_.x);
  context_continuous_state.set_y(initial_conditions_.y);
  context_continuous_state.set_heading(initial_conditions_.heading);
  context_continuous_state.set_velocity(initial_conditions_.speed);

  /*********************
   * Unicycle Car System
   *********************/
  typedef UnicycleCar<double> UnicycleCarSystem;
  UnicycleCarSystem* simple_car_system =
      builder.AddSystem(std::make_unique<UnicycleCarSystem>(context_continuous_state));
  simple_car_system->set_name(this->name() + "_system");

  /*********************
   * Teleop Systems
   *********************/
  std::string command_channel = "teleop/" + this->name();
  typedef IgnSubscriberSystem<ignition::msgs::AutomotiveAngularRateAccelerationCommand>
      AngularRateAccelerationCommandSubscriber;
  AngularRateAccelerationCommandSubscriber* input_command_subscriber =
      builder.AddSystem(std::make_unique<AngularRateAccelerationCommandSubscriber>(command_channel));

  auto input_command_translator = builder.AddSystem(std::make_unique<IgnAngularRateAccelerationCommandToDrake>());

  // Ignition anglular rate and acceleration commands received through the subscriber are translated
  // to Drake.
  builder.Connect(*input_command_subscriber, *input_command_translator);

  // And then the translated Drake command is sent to the car.
  builder.Connect(*input_command_translator, *simple_car_system);

  /*********************
   * Diagram Outputs
   *********************/
  builder.ExportStateOutput(simple_car_system->state_output());
  builder.ExportPoseOutput(simple_car_system->pose_output());
  builder.ExportVelocityOutput(simple_car_system->velocity_output());

  return builder.Build();
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
