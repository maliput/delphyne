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
#include "backend/translation_systems/drake_simple_car_state_to_ign.h"
#include "backend/translation_systems/ign_driving_command_to_drake.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

SimpleCar::SimpleCar(const std::string& name, double x, double y,
                     double heading, double velocity)
    : delphyne::Agent(name),
      simple_car_state_(std::make_unique<SimpleCarState>()),
      simple_car_system_() {
  igndbg << "SimpleCar constructor" << std::endl;
  simple_car_state_->set_x(x);
  simple_car_state_->set_y(y);
  simple_car_state_->set_heading(heading);
  simple_car_state_->set_velocity(velocity);
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
   * Instantiate System
   *********************/
  std::unique_ptr<drake::automotive::SimpleCar2<double>> system =
      std::make_unique<drake::automotive::SimpleCar2<double>>();
  system->set_name(name_);
  simple_car_system_ =
      builder->template AddSystem<drake::automotive::SimpleCar2<double>>(
          std::move(system));

  /*********************
   * Teleop
   *********************/
  std::string command_channel = "teleop/" + std::to_string(id);
  auto driving_command_subscriber = builder->template AddSystem<
      IgnSubscriberSystem<ignition::msgs::AutomotiveDrivingCommand>>(
      command_channel);

  auto driving_command_translator =
      builder->template AddSystem<IgnDrivingCommandToDrake>();

  // Ignition driving commands received through the subscriber are translated
  // to Drake.
  builder->Connect(*driving_command_subscriber, *driving_command_translator);

  // And then the translated Drake command is sent to the car.
  builder->Connect(*driving_command_translator, *simple_car_system_);

  /*********************
   * Diagram Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  auto ports = aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(simple_car_system_->pose_output(), ports.pose_descriptor);
  builder->Connect(simple_car_system_->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id, name_));

  /*********************
   * Other
   *********************/
  auto agent_state_translator =
      builder->template AddSystem<DrakeSimpleCarStateToIgn>();

  const std::string car_state_channel =
      "agents/" + std::to_string(id) + "/state";
  auto car_state_publisher = builder->template AddSystem<
      IgnPublisherSystem<ignition::msgs::SimpleCarState>>(car_state_channel);

  // Drake car states are translated to ignition.
  builder->Connect(simple_car_system_->state_output(),
                   agent_state_translator->get_input_port(0));

  // And then the translated ignition car state is published.
  builder->Connect(*agent_state_translator, *car_state_publisher);

  return 0;
}

int SimpleCar::Initialize(drake::systems::Context<double>* context) {
  igndbg << "SimpleCar initialize" << std::endl;

  drake::systems::VectorBase<double>& context_state =
      context->get_mutable_continuous_state().get_mutable_vector();
  drake::systems::BasicVector<double>* const state =
      dynamic_cast<drake::systems::BasicVector<double>*>(&context_state);
  DELPHYNE_ASSERT(state);
  state->set_value(simple_car_state_->get_value());

  return 0;
}

drake::systems::System<double>* SimpleCar::get_system() const {
  return simple_car_system_;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
