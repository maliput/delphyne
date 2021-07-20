/**
 * @file src/agents/unicycle_car
 *
 * Copyright 2020 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/unicycle_car.h"

#include <string>
#include <utility>

#include <drake/systems/primitives/multiplexer.h>
#include <maliput/common/maliput_unused.h>

#include "gen/angular_rate_acceleration_command.h"
#include "gen/simple_car_state.h"
#include "systems/unicycle_car.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

UnicycleCarBlueprint::UnicycleCarBlueprint(const std::string& name, double x, double y, double heading, double speed)
    : TypedAgentBlueprint<UnicycleCarAgent>(name), initial_conditions_(x, y, heading, speed) {}

std::unique_ptr<UnicycleCarAgent> UnicycleCarBlueprint::DoBuildAgentInto(
    const maliput::api::RoadGeometry* road_geometry, drake::systems::DiagramBuilder<double>* simulator_builder) const {
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
  UnicycleCarSystem* unicycle_car_system =
      builder.AddSystem(std::make_unique<UnicycleCarSystem>(context_continuous_state));
  unicycle_car_system->set_name(this->name() + "_system");

  auto acceleration_setter = builder.template AddSystem(std::make_unique<delphyne::VectorSource<double>>(-1));
  auto angular_rate_setter = builder.template AddSystem(std::make_unique<delphyne::VectorSource<double>>(-1));

  drake::systems::Multiplexer<double>* mux = builder.AddSystem<drake::systems::Multiplexer<double>>(
      std::make_unique<drake::systems::Multiplexer<double>>(AngularRateAccelerationCommand<double>()));
  mux->set_name(this->name() + "_mux");

  builder.Connect(angular_rate_setter->output(), mux->get_input_port(0));
  builder.Connect(acceleration_setter->output(), mux->get_input_port(1));
  builder.Connect(mux->get_output_port(0), unicycle_car_system->get_input_port(0));

  /*********************
   * Diagram Outputs
   *********************/
  builder.ExportStateOutput(unicycle_car_system->state_output());
  builder.ExportPoseOutput(unicycle_car_system->pose_output());
  builder.ExportVelocityOutput(unicycle_car_system->velocity_output());

  Agent::Diagram* agent_diagram = simulator_builder->AddSystem(builder.Build());
  return std::make_unique<UnicycleCarAgent>(agent_diagram, acceleration_setter, angular_rate_setter);
}

}  // namespace delphyne
