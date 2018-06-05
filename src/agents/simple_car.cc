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

#include <drake/automotive/prius_vis.h>
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/gen/simple_car_state_translator.h"

#include "backend/ign_publisher_system.h"
#include "backend/ign_subscriber_system.h"
#include "translations/drake_simple_car_state_to_ign.h"
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
  igndbg << "SimpleCar constructor" << std::endl;
}

int SimpleCar::Configure(
    int id, const drake::maliput::api::RoadGeometry* road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "SimpleCar configure" << std::endl;

  /*********************
   * Basics
   *********************/
  id_ = id;

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
      builder->template AddSystem<SimpleCarSystem>(
          std::make_unique<SimpleCarSystem>(context_continuous_state,
                                            context_numeric_parameters));
  simple_car_system->set_name(name_);

  /*********************
   * Teleop Systems
   *********************/
  std::string command_channel = "teleop/" + std::to_string(id);
  typedef IgnSubscriberSystem<ignition::msgs::AutomotiveDrivingCommand>
      DrivingCommandSubscriber;
  DrivingCommandSubscriber* driving_command_subscriber =
      builder->template AddSystem<DrivingCommandSubscriber>(
          std::make_unique<DrivingCommandSubscriber>(command_channel));

  auto driving_command_translator =
      builder->template AddSystem<IgnDrivingCommandToDrake>();

  // Ignition driving commands received through the subscriber are translated
  // to Drake.
  builder->Connect(*driving_command_subscriber, *driving_command_translator);

  // And then the translated Drake command is sent to the car.
  builder->Connect(*driving_command_translator, *simple_car_system);

  /*********************
   * Simulator Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  drake::systems::rendering::PoseVelocityInputPortDescriptors<double> ports =
      aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(simple_car_system->pose_output(), ports.pose_descriptor);
  builder->Connect(simple_car_system->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id, name_));

  /*********************
   * State Publisher
   *********************/
  auto agent_state_translator =
      builder->template AddSystem<DrakeSimpleCarStateToIgn>();

  const std::string agent_state_channel =
      "agents/" + std::to_string(id) + "/state";
  typedef IgnPublisherSystem<ignition::msgs::SimpleCarState>
      AgentStatePublisherSystem;
  AgentStatePublisherSystem* agent_state_publisher_system =
      builder->template AddSystem<AgentStatePublisherSystem>(
          std::make_unique<AgentStatePublisherSystem>(agent_state_channel));

  // Drake car states are translated to ignition.
  builder->Connect(simple_car_system->state_output(),
                   agent_state_translator->get_input_port(0));

  // And then the translated ignition car state is published.
  builder->Connect(*agent_state_translator, *agent_state_publisher_system);

  return 0;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
