/**
 * @file src/agents/rail_car.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/rail_car.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <drake/automotive/maliput/api/junction.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/api/segment.h>
#include <drake/common/eigen_types.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/primitives/constant_vector_source.h>

// public headers
#include "delphyne/macros.h"
#include "delphyne/maliput/find_lane.h"

#include "gen/rail_follower_params.h"
#include "gen/rail_follower_state.h"
#include "gen/simple_car_state_translator.h"

// private headers
#include "systems/lane_direction.h"
#include "systems/rail_follower.h"
#include "systems/speed_system.h"
#include "systems/vector_source.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

RailCarBlueprint::RailCarBlueprint(const std::string& name,
                                   const drake::maliput::api::Lane& lane,
                                   bool direction_of_travel,
                                   double longitudinal_position,  // s
                                   double lateral_offset,         // r
                                   double speed, double nominal_speed)
    : AgentBlueprint(name), initial_parameters_(
          lane, direction_of_travel, longitudinal_position,
          lateral_offset, speed, nominal_speed) {
  /*********************
   * Checks
   *********************/
  DELPHYNE_VALIDATE(
      initial_parameters_.lane.segment(), std::invalid_argument,
      "The lane to be initialised on is not part of a road segment "
      "(subsequently road geometry).");
  DELPHYNE_VALIDATE(
      initial_parameters_.lane.segment()->junction(), std::invalid_argument,
      "The lane to be initialised on is not connected to a junction "
      "(subsequently road geometry).");
  DELPHYNE_VALIDATE(
      initial_parameters_.lane.segment()->junction()->road_geometry(),
      std::invalid_argument,
      "The lane to be initialised on is not part of a road geometry.");

  drake::maliput::api::LanePosition initial_car_lane_position{
      longitudinal_position, lateral_offset, 0.0};
  drake::maliput::api::GeoPosition initial_car_geo_position =
      lane.ToGeoPosition(initial_car_lane_position);
  drake::maliput::api::Rotation initial_car_orientation =
      lane.GetOrientation(initial_car_lane_position);
  SetInitialWorldPose(drake::Translation3<double>(
      initial_car_geo_position.xyz()) * initial_car_orientation.quat());
}

std::unique_ptr<RailCar> RailCarBlueprint::BuildInto(
    const drake::maliput::api::RoadGeometry* road_geometry,
    drake::systems::DiagramBuilder<double>* builder) const {
  DELPHYNE_VALIDATE(road_geometry != nullptr, std::invalid_argument,
                    "Rail cars need a road geometry to drive on, make "
                    "sure the simulation is built with one.");
  const drake::maliput::api::RoadGeometry* lane_road_geometry =
      initial_parameters_.lane.segment()->junction()->road_geometry();
  DELPHYNE_VALIDATE(
      lane_road_geometry->id() == road_geometry->id(),
      std::invalid_argument,
      "The provided initial lane is not on the same road geometry "
      "as that used by the simulation");
  DELPHYNE_VALIDATE(
      maliput::FindLane(initial_parameters_.lane.id(), *road_geometry),
      std::invalid_argument,
      "The provided initial lane is not within this simulation's "
      "RoadGeometry.");

  /******************************************
   * Initial Context Variables
   ******************************************/
  typedef RailFollowerState<double> ContextContinuousState;
  typedef RailFollowerParams<double> ContextNumericParameters;

  ContextContinuousState context_continuous_state;
  context_continuous_state.set_s(initial_parameters_.position);
  context_continuous_state.set_speed(initial_parameters_.speed);
  ContextNumericParameters context_numeric_parameters;
  context_numeric_parameters.set_r(initial_parameters_.offset);
  context_numeric_parameters.set_h(0.0);
  context_numeric_parameters.set_max_speed(initial_parameters_.nominal_speed);

  /*********************
   * Instantiate System
   *********************/
  AgentBlueprint::DiagramBuilder agent_diagram_builder(this->name());

  // TODO(daniel.stonier): LaneDirection is discombabulating, it is more than
  // just direction and not even expressive of the lane's direction. Rather
  // it is intended to be used by the car (not it is not maliput api) to
  // indicate where in the road network it is and whether it is desired to
  // travel against the flow the lane's nominal direction (traffic flow).
  // Probably preferable to not use this at all and specify things separately.
  LaneDirection lane_direction(&(initial_parameters_.lane),
                               initial_parameters_.direction_of_travel);
  RailFollower<double>* rail_follower_system =
      agent_diagram_builder.AddSystem(std::make_unique<RailFollower<double>>(
          lane_direction, context_continuous_state,
          context_numeric_parameters));
  rail_follower_system->set_name(name() + "_system");

  auto speed_setter = agent_diagram_builder.template AddSystem(
      std::make_unique<delphyne::VectorSource<double>>(-1));

  delphyne::SpeedSystem<double>* speed_system =
      agent_diagram_builder.AddSystem(std::make_unique<delphyne::SpeedSystem<double>>());

  agent_diagram_builder.Connect(speed_system->acceleration_output(),
                  rail_follower_system->command_input());

  agent_diagram_builder.Connect(rail_follower_system->velocity_output(),
                  speed_system->feedback_input());

  agent_diagram_builder.Connect(speed_setter->output(), speed_system->command_input());

  /*********************
   * Diagram Outputs
   *********************/
  agent_diagram_builder.ExportStateOutput(rail_follower_system->simple_car_state_output());
  agent_diagram_builder.ExportPoseOutput(rail_follower_system->pose_output());
  agent_diagram_builder.ExportVelocityOutput(rail_follower_system->velocity_output());

  Agent::Diagram* agent_diagram = builder->AddSystem(agent_diagram_builder.Build());
  return std::make_unique<RailCar>(agent_diagram, speed_setter);
}

RailCar::RailCar(Agent::Diagram* diagram, VectorSource<double>* speed_setter)
    : Agent(diagram), speed_setter_(speed_setter) {
}

void RailCar::SetSpeed(double new_speed_mps) {
  speed_setter_->Set(new_speed_mps);
}

}  // namespace delphyne
