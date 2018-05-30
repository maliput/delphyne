/**
 * @file src/agents/mobil_car.h
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
#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "delphyne/agent_base.h"
#include "systems/simple_car.h"

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
 */
class MobilCar : public delphyne::Agent {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilCar)

  /**
   * @brief Default constructor
   *
   * @param name: unique name for the agent
   * @param direction_of_travel: designates whether the car will travel
   *            with or against the flow specified by the lane's rules
    * @param x: scene x-coordinate
    * @param y: scene y-coordinate
    * @param heading: orientation of the car in the x-y frame
    * @param speed: actual initial speed
   */
  MobilCar(const std::string& name, const bool& direction_of_travel,
           const double& x, const double& y, const double& heading,
           const double& speed);

  int Configure(
      int id,
      const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<double>* builder,
      drake::systems::rendering::PoseAggregator<double>* aggregator,
      drake::automotive::CarVisApplicator<double>* car_vis_applicator) override;

  int Initialize(drake::systems::Context<double>* system_context) override;

  drake::systems::System<double>* get_system() const;

 private:
  struct Parameters {
    bool direction_of_travel{true};
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double offset{0.0};
    double speed{0.0};
    Parameters(const bool& direction_of_travel, const double& x,
               const double& y, const double& heading, const double& speed)
        : direction_of_travel(direction_of_travel),
          x(x),
          y(y),
          heading(heading),
          speed(speed) {}
  } initial_parameters_;

  typedef drake::automotive::SimpleCar2<double> SimpleCarSystem;
  SimpleCarSystem* simple_car_system_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne

