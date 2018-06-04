/**
 * @file src/agents/rail_car.h
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <drake/automotive/car_vis_applicator.h>
#include <drake/automotive/gen/maliput_railcar_state.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

// public headers
#include "delphyne/agent_base.h"

// private headers
#include "systems/maliput_rail_car.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// @brief An agent that follows roads as if they were railroad tracks.
///
/// The underlying road network has a reference line for each lane which
/// is utilised by this agent as a railroad track.
///
/// Initial position is specified in the lane longitudinal co-ordinate
/// (how far along the track) and the agent will follow
/// this track exactly - the only variance it is permitted is the speed
/// with which it follows the track.
class RailCar : public delphyne::Agent {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RailCar)

  ///
  /// @brief Default constructor
  ///
  /// @param name[in] Unique name for the agent.
  /// @param lane[in] The lane to start in.
  /// @param direction_of_travel[in] Designates whether the car will travel
  ///            with or against the flow specified by the lane's rules
  /// @param longitudinal_position[in] Initial position on the lane's track
  ///            (maliput lane coordinate, 's' (m)).
  /// @param lateral_offset[in] The offset perpendicular to the centre road
  ///            line (maliput lane coordinate 'r' (m)).
  /// @param speed[in] The actual initial speed.
  /// @param nominal_speed[in] The desired cruising speed.
  RailCar(const std::string& name, const drake::maliput::api::Lane& lane,
          bool direction_of_travel,
          double longitudinal_position,  // s
          double lateral_offset,         // r
          double speed, double nominal_speed);

  int Configure(
      int id, const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<double>* builder,
      drake::systems::rendering::PoseAggregator<double>* aggregator,
      drake::automotive::CarVisApplicator<double>* car_vis_applicator) override;

  int Initialize(drake::systems::Context<double>* context) override;

  drake::systems::System<double>* get_system() const;

 private:
  // Container for the agent's initial configuration.
  //
  // Note: this is independent of whatever computational mechanisms
  // are used internally and is a useful construct for recording and
  // logging / streaming to debug configuration errors.
  struct Parameters {
    const drake::maliput::api::Lane& lane;
    bool direction_of_travel{true}; // with or against the lane s-axis
    double position{0.0};       // longitudinal position in lane (m)
    double offset{0.0};         // lateral position in lane (m)
    double speed{0.0};          // speed in direction of the lane s-axis (m/s)
    double nominal_speed{0.0};  // nominal cruising speed (m/s)
    Parameters(const drake::maliput::api::Lane& lane, bool direction_of_travel,
               double position,  // s
               double offset,    // r
               double speed, double nominal_speed)
        : lane(lane),
          direction_of_travel(direction_of_travel),
          position(position),
          offset(offset),
          speed(speed),
          nominal_speed(nominal_speed) {}
  } initial_parameters_;

  typedef drake::automotive::MaliputRailCar<double> RailCarSystem;

  RailCarSystem* rail_car_system_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
