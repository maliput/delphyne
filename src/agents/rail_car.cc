/**
 * @file src/agents/rail_car.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/rail_car.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <experimental/optional>

#include <drake/automotive/gen/maliput_railcar_params.h>
#include <drake/automotive/gen/simple_car_state_translator.h>
#include <drake/automotive/lane_direction.h>
#include <drake/automotive/maliput/api/junction.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/api/segment.h>
#include <drake/automotive/prius_vis.h>
#include <drake/common/eigen_types.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/rendering/pose_aggregator.h>

// public headers
#include "delphyne/macros.h"
#include "delphyne/maliput/find_lane.h"

// private headers
#include "agents/helpers/geometry_wiring.h"
#include "backend/ign_publisher_system.h"
#include "backend/ign_subscriber_system.h"
#include "systems/maliput_rail_car.h"
#include "translations/drake_simple_car_state_to_ign.h"
#include "translations/ign_driving_command_to_drake.h"
#include "translations/pose_and_vel_to_simple_car_state.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

RailCar::RailCar(const std::string& name, const drake::maliput::api::Lane& lane,
                 bool direction_of_travel,
                 double longitudinal_position,  // s
                 double lateral_offset,         // r
                 double speed, double nominal_speed)
    : delphyne::Agent(name),
      initial_parameters_(lane, direction_of_travel, longitudinal_position,
                          lateral_offset, speed, nominal_speed) {
  igndbg << "RailCar constructor" << std::endl;
}

void RailCar::Configure(
    int id, const drake::maliput::api::RoadGeometry* road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "RailCar configure" << std::endl;

  /*********************
   * Checks
   *********************/
  DELPHYNE_VALIDATE(builder != nullptr, std::invalid_argument,
                    "Builder must not be null");
  DELPHYNE_VALIDATE(scene_graph != nullptr, std::invalid_argument,
                    "Scene graph must not be null");
  DELPHYNE_VALIDATE(aggregator != nullptr, std::invalid_argument,
                    "Aggregator must not be null");
  DELPHYNE_VALIDATE(car_vis_applicator != nullptr, std::invalid_argument,
                    "Car visualization applicator must not be null");
  DELPHYNE_VALIDATE(road_geometry != nullptr, std::invalid_argument,
                    "Rail cars need a road geometry to drive on, make "
                    "sure the simulation is configured with one.");

  DELPHYNE_VALIDATE(initial_parameters_.lane.segment(), std::runtime_error,
                    "The lane to be initialised on is not part of a "
                    "road segment (subsequently road geometry).");
  DELPHYNE_VALIDATE(initial_parameters_.lane.segment()->junction(),
                    std::runtime_error,
                    "The lane to be initialised on is not connected to "
                    "a junction (subsequently road geometry).");
  DELPHYNE_VALIDATE(
      initial_parameters_.lane.segment()->junction()->road_geometry(),
      std::runtime_error,
      "The lane to be initialised on is not part of a road geometry.");
  DELPHYNE_VALIDATE(
      initial_parameters_.lane.segment()->junction()->road_geometry()->id() ==
          road_geometry->id(),
      std::runtime_error,
      "The provided initial lane is not on the same road geometry as that used "
      "by the simulation");
  DELPHYNE_VALIDATE(
      maliput::FindLane(initial_parameters_.lane.id(), *road_geometry),
      std::runtime_error,
      "The provided initial lane is not within this simulation's "
      "RoadGeometry.");

  /*********************
   * Basics
   *********************/
  id_ = id;

  /******************************************
   * Initial Context Variables
   ******************************************/
  typedef drake::automotive::MaliputRailcarState<double> ContextContinuousState;
  typedef drake::automotive::MaliputRailcarParams<double>
      ContextNumericParameters;
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
  // TODO(daniel.stonier): LaneDirection is discombabulating, it is more than
  // just direction and not even expressive of the lane's direction. Rather
  // it is intended to be used by the car (not it is not maliput api) to
  // indicate where in the road network it is and whether it is desired to
  // travel against the flow the lane's nominal direction (traffic flow).
  // Probably preferable to not use this at all and specify things separately.
  drake::automotive::LaneDirection lane_direction(
      &(initial_parameters_.lane), initial_parameters_.direction_of_travel);
  typedef drake::automotive::MaliputRailCar<double> RailCarSystem;
  RailCarSystem* rail_car_system = builder->template AddSystem<RailCarSystem>(
      std::make_unique<RailCarSystem>(lane_direction, context_continuous_state,
                                      context_numeric_parameters));
  rail_car_system->set_name(name_);

  /*********************
   * Simulator Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  drake::systems::rendering::PoseVelocityInputPortDescriptors<double> ports =
      aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(rail_car_system->pose_output(), ports.pose_descriptor);
  builder->Connect(rail_car_system->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id_, name_));

  drake::maliput::api::LanePosition initial_car_lane_position{
    initial_parameters_.position, initial_parameters_.offset, 0.};
  drake::maliput::api::GeoPosition initial_car_geo_position =
      initial_parameters_.lane.ToGeoPosition(initial_car_lane_position);
  drake::maliput::api::Rotation initial_car_orientation =
      initial_parameters_.lane.GetOrientation(initial_car_lane_position);

  // Computes the initial world to car transform X_WC0.
  const drake::Isometry3<double> X_WC0 =
      drake::Translation3<double>(initial_car_geo_position.xyz())
      * initial_car_orientation.quat();

  // Wires up the Prius geometry.
  builder->Connect(rail_car_system->pose_output(), WirePriusGeometry(
      name_, X_WC0, builder, scene_graph, &geometry_ids_));

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

  auto pose_and_vel_to_simple_car_state =
      builder->template AddSystem<PoseAndVelToSimpleCarState>();

  // Connects the railcar's pose and velocity outputs to the simple car
  // state calclator system.
  builder->Connect(rail_car_system->pose_output(),
                   pose_and_vel_to_simple_car_state->get_input_port(0));
  builder->Connect(rail_car_system->velocity_output(),
                   pose_and_vel_to_simple_car_state->get_input_port(1));

  // Connects the simple car state calculator system's output to the ignition
  // publisher.
  builder->Connect(pose_and_vel_to_simple_car_state->simple_car_state_output(),
                   agent_state_translator->get_input_port(0));

  // And then the translated ignition car state is published.
  builder->Connect(*agent_state_translator, *agent_state_publisher_system);
}

}  // namespace delphyne
