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

#include "delphyne/agent_base.h"
#include "systems/maliput_railcar.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief An agent that follows roads as if they were railroad tracks.
 *
 * The underlying road network has a reference line for each lane which
 * is utilised by this agent as a railroad track.
 *
 * Initial position is specified in the lane longitudinal co-ordinate
 * (how far along the track) and the agent will follow
 * this track exactly - the only variance it is permitted is the speed
 * with which it follows the track.
 *
 * TODO(daniel.stonier): enable nominal_speed as a dynamically
 * configurable variable from the scenario (e.g. from python scriptlets)
 */
class RailCar : public delphyne::Agent {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RailCar)

  /**
   * @brief Default constructor
   *
   * @param name: unique name for the agent
   * @param lane: the lane to start in
   * @param direction_of_travel: designates whether the car will travel
   *            with or against the flow specified by the lane's rules
   * @param longitudinal_position: initial position on the lane's track
   *            (maliput lane coordinate, 's' (m))
   * @param lateral_offset: offset perpendicular to the centre road line
   *            (maliput lane coordinate 'r' (m))
   * @param speed: actual initial speed
   * @param nominal_speed: desired cruising speed
   */
  RailCar(const std::string& name, const drake::maliput::api::Lane& lane,
          const bool& direction_of_travel,
          const double& longitudinal_position,  // s
          const double& lateral_offset,         // r
          const double& speed, const double& nominal_speed);

  int Configure(
      int id, const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<double>* builder,
      drake::systems::rendering::PoseAggregator<double>* aggregator,
      drake::automotive::CarVisApplicator<double>* car_vis_applicator) override;

  int Initialize(drake::systems::Context<double>* context) override;

  drake::systems::System<double>* get_system() const;

 private:
  struct Parameters {
    const drake::maliput::api::Lane& lane;
    bool direction_of_travel{true};
    double position{0.0};
    double offset{0.0};
    double speed{0.0};
    double nominal_speed{0.0};
    Parameters(const drake::maliput::api::Lane& lane,
               const bool& direction_of_travel,
               const double& position,  // s
               const double& offset,    // r
               const double& speed, const double& nominal_speed)
        : lane(lane),
          direction_of_travel(direction_of_travel),
          position(position),
          offset(offset),
          speed(speed),
          nominal_speed(nominal_speed) {}
  } initial_parameters_;

  typedef drake::automotive::MaliputRailcarState<double> RailCarContextState;
  typedef drake::automotive::MaliputRailcarParams<double>
      RailCarContextParameters;
  typedef drake::automotive::MaliputRailcar2<double> RailCarSystem;

  std::unique_ptr<RailCarContextState> rail_car_context_state_;
  std::unique_ptr<RailCarContextParameters> rail_car_context_parameters_;
  RailCarSystem* rail_car_system_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
