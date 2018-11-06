/**
 * @file src/agents/simple_car
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/simple_car.h"

#include <string>
#include <utility>

#include "backend/ign_subscriber_system.h"
#include "gen/simple_car_state.h"
#include "gen/simple_car_state_translator.h"
#include "translations/ign_driving_command_to_drake.h"
#include "systems/simple_car.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

SimpleCarBlueprint::SimpleCarBlueprint(const std::string& name, double x,
                                       double y, double heading, double speed)
    : SimpleAgentBlueprint(name, drake::Isometry3<double>(
          drake::Translation3<double>(x, y, 0.0) *
          drake::AngleAxis<double>(heading, drake::Vector3<double>::UnitZ()))),
      initial_parameters_(x, y, heading, speed) {
}

std::unique_ptr<Agent::Diagram> SimpleCarBlueprint::DoBuildDiagram(
    const drake::maliput::api::RoadGeometry* road_geometry) const {
  drake::unused(road_geometry);

  AgentBlueprint::DiagramBuilder builder(this->name());

  /*********************
   * Context
   *********************/
  typedef SimpleCarState<double> ContextContinuousState;
  typedef SimpleCarParams<double> ContextNumericParameters;
  ContextContinuousState context_continuous_state;
  context_continuous_state.set_x(initial_parameters_.x);
  context_continuous_state.set_y(initial_parameters_.y);
  context_continuous_state.set_heading(initial_parameters_.heading);
  context_continuous_state.set_velocity(initial_parameters_.speed);
  ContextNumericParameters context_numeric_parameters;

  /*********************
   * Simple Car System
   *********************/
  typedef SimpleCar2<double> SimpleCarSystem;
  SimpleCarSystem* simple_car_system =
      builder.AddSystem(std::make_unique<SimpleCarSystem>(
          context_continuous_state, context_numeric_parameters));
  simple_car_system->set_name(this->name() + "_system");

  /*********************
   * Teleop Systems
   *********************/
  std::string command_channel = "teleop/" + this->name();
  typedef IgnSubscriberSystem<ignition::msgs::AutomotiveDrivingCommand>
      DrivingCommandSubscriber;
  DrivingCommandSubscriber* driving_command_subscriber = builder.AddSystem(
      std::make_unique<DrivingCommandSubscriber>(command_channel));

  auto driving_command_translator =
      builder.AddSystem(std::make_unique<IgnDrivingCommandToDrake>());

  // Ignition driving commands received through the subscriber are translated
  // to Drake.
  builder.Connect(*driving_command_subscriber, *driving_command_translator);

  // And then the translated Drake command is sent to the car.
  builder.Connect(*driving_command_translator, *simple_car_system);

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
