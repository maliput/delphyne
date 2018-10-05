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

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

SimpleCar::SimpleCar(const std::string& name, double x, double y,
                     double heading, double speed)
    : delphyne::Agent(name), initial_parameters_(x, y, heading, speed) {
  // TODO(daniel.stonier) stop using this, make use of an initial value on
  // the pose output
  initial_world_pose_ = drake::Isometry3<double>(
      drake::Translation3<double>(initial_parameters_.x, initial_parameters_.y,
                                  0.0) *
      drake::AngleAxis<double>(initial_parameters_.heading,
                               drake::Vector3<double>::UnitZ()));
}

std::unique_ptr<Agent::DiagramBundle> SimpleCar::BuildDiagram() const {
  DiagramBuilder builder(name_);

  /*********************
   * Context
   *********************/
  typedef drake::automotive::SimpleCarState<double> ContextContinuousState;
  typedef drake::automotive::SimpleCarParams<double> ContextNumericParameters;
  ContextContinuousState context_continuous_state;
  context_continuous_state.set_x(initial_parameters_.x);
  context_continuous_state.set_y(initial_parameters_.y);
  context_continuous_state.set_heading(initial_parameters_.heading);
  context_continuous_state.set_velocity(initial_parameters_.speed);
  ContextNumericParameters context_numeric_parameters;

  /*********************
   * Simple Car System
   *********************/
  typedef drake::automotive::SimpleCar2<double> SimpleCarSystem;
  SimpleCarSystem* simple_car_system =
      builder.AddSystem(std::make_unique<SimpleCarSystem>(
          context_continuous_state, context_numeric_parameters));
  simple_car_system->set_name(name_);

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

  return std::move(builder.Build());
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
