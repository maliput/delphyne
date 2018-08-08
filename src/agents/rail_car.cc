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

#include <drake/automotive/gen/simple_car_state_translator.h>
#include <drake/automotive/lane_direction.h>
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

// private headers
#include "systems/rail_follower.h"
#include "systems/rail_follower_params.h"
#include "systems/rail_follower_state.h"
#include "systems/speed_system.h"

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
                 double speed, double nominal_speed,
                 const drake::maliput::api::RoadGeometry& road_geometry)
    : delphyne::Agent(name),
      initial_parameters_(lane, direction_of_travel, longitudinal_position,
                          lateral_offset, speed, nominal_speed),
      road_geometry_(road_geometry) {
  /*********************
   * Initial World Pose
   *********************/
  drake::maliput::api::LanePosition initial_car_lane_position{
      initial_parameters_.position, initial_parameters_.offset, 0.};
  drake::maliput::api::GeoPosition initial_car_geo_position =
      initial_parameters_.lane.ToGeoPosition(initial_car_lane_position);
  drake::maliput::api::Rotation initial_car_orientation =
      initial_parameters_.lane.GetOrientation(initial_car_lane_position);
  initial_world_pose_ =
      drake::Translation3<double>(initial_car_geo_position.xyz()) *
      initial_car_orientation.quat();

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
  DELPHYNE_VALIDATE(
      initial_parameters_.lane.segment()->junction()->road_geometry()->id() ==
          road_geometry.id(),
      std::invalid_argument,
      "The provided initial lane is not on the same road geometry "
      "as that used by the simulation");
  DELPHYNE_VALIDATE(
      maliput::FindLane(initial_parameters_.lane.id(), road_geometry),
      std::invalid_argument,
      "The provided initial lane is not within this simulation's "
      "RoadGeometry.");
}

std::unique_ptr<Agent::DiagramBundle> RailCar::BuildDiagram() const {
  DiagramBuilder builder(name_);

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
  // TODO(daniel.stonier): LaneDirection is discombabulating, it is more than
  // just direction and not even expressive of the lane's direction. Rather
  // it is intended to be used by the car (not it is not maliput api) to
  // indicate where in the road network it is and whether it is desired to
  // travel against the flow the lane's nominal direction (traffic flow).
  // Probably preferable to not use this at all and specify things separately.
  drake::automotive::LaneDirection lane_direction(
      &(initial_parameters_.lane), initial_parameters_.direction_of_travel);
  RailFollower<double>* rail_follower_system =
      builder.AddSystem(std::make_unique<RailFollower<double>>(
          lane_direction, context_continuous_state,
          context_numeric_parameters));
  rail_follower_system->set_name(name_ + "_system");

  speed_system_ = builder.AddSystem(
      std::make_unique<delphyne::SpeedSystem<double>>());

  builder.Connect(speed_system_->acceleration_output(),
                  rail_follower_system->command_input());

  builder.Connect(rail_follower_system->velocity_output(),
                  speed_system_->feedback_input());

  /*********************
   * Diagram Outputs
   *********************/
  builder.ExportStateOutput(rail_follower_system->simple_car_state_output());
  builder.ExportPoseOutput(rail_follower_system->pose_output());
  builder.ExportVelocityOutput(rail_follower_system->velocity_output());

  return std::move(builder.Build());
}

void RailCar::SetSpeed(double new_speed_mps) {
  speed_system_->SetSpeed(new_speed_mps);
}

}  // namespace delphyne
