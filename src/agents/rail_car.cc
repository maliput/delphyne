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
#include <drake/systems/framework/context.h>
#include <drake/systems/rendering/pose_aggregator.h>

// public headers
#include "delphyne/maliput/find_lane.h"

// private headers
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

RailCar::RailCar(const std::string& name, const drake::maliput::api::Lane& lane,
                 bool direction_of_travel,
                 double longitudinal_position,  // s
                 double lateral_offset,         // r
                 double speed, double nominal_speed)
    : delphyne::Agent(name),
      initial_parameters_(lane, direction_of_travel, longitudinal_position,
                          lateral_offset, speed, nominal_speed),
      rail_car_system_(nullptr) {
  igndbg << "RailCar constructor" << std::endl;
}

int RailCar::Configure(
    int id, const drake::maliput::api::RoadGeometry* road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "RailCar configure" << std::endl;

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
  if (!initial_parameters_.lane.segment()) {
    ignerr << "The lane to be initialised on is not part of a road segment "
           << "(subsequently road geometry)." << std::endl;
    return -1;
  }
  if (!initial_parameters_.lane.segment()->junction()) {
    ignerr << "The lane to be initialised on is not connected to a junction "
           << "(subsequently road geometry)." << std::endl;
    return -1;
  }
  if (!initial_parameters_.lane.segment()->junction()->road_geometry()) {
    ignerr << "The lane to be initialised on is not part of a road geometry."
           << std::endl;
    return -1;
  }
  if (initial_parameters_.lane.segment()->junction()->road_geometry()->id() !=
      road_geometry->id()) {
    ignerr << "RailCar::Configure(): "
              "The provided initial lane is not on the same road geometry "
              "as that used by the simulation"
           << std::endl;
    return -1;
  }
  if (!maliput::FindLane(initial_parameters_.lane.id(), *road_geometry)) {
    ignerr << "RailCar::Configure(): "
              "The provided initial lane is not within this simulation's "
              "RoadGeometry."
           << std::endl;
    return -1;
  }

  /******************************************
   * Initial Context Variables
   ******************************************/
  typedef drake::automotive::MaliputRailcarState<double> ContextContinuousState;
  typedef drake::automotive::MaliputRailcarParams<double> ContextNumericParameters;
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
      &(initial_parameters_.lane),
      initial_parameters_.direction_of_travel);
  std::unique_ptr<RailCarSystem> system =
      std::make_unique<RailCarSystem>(
          lane_direction,
          context_continuous_state,
          context_numeric_parameters);
  system->set_name(name_);
  rail_car_system_ =
      builder->template AddSystem<RailCarSystem>(std::move(system));

  /*********************
   * Diagram Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  drake::systems::rendering::PoseVelocityInputPortDescriptors<double> ports =
      aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(rail_car_system_->pose_output(), ports.pose_descriptor);
  builder->Connect(rail_car_system_->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id_, name_));

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
  builder->Connect(rail_car_system_->simple_car_state_output(),
                   agent_state_translator->get_input_port(0));

  // And then the translated ignition car state is published.
  builder->Connect(*agent_state_translator, *car_state_publisher);

  return 0;
}

int RailCar::Initialize(drake::systems::Context<double>* system_context) {
  // TODO(daniel.stonier) deprecate this method once all agents
  // have shifted to pre-declaring their context on system construction
  // (see Configure().
    return 0;
}

drake::systems::System<double>* RailCar::get_system() const {
  return rail_car_system_;
}

}  // namespace delphyne
