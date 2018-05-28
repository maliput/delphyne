/**
 * @file src/agents/simple_car.h
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
#include <drake/automotive/gen/simple_car_state.h>
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
 * @brief A very simple vehicle agent that can be teleoperated.
 */
class SimpleCar : public delphyne::Agent {
 public:
  SimpleCar(const std::string& name, const double& x, const double& y,
            const double& heading, const double& velocity);
  int Configure(
      const int& id, const drake::maliput::api::RoadGeometry& road_geometry,
      drake::systems::DiagramBuilder<double>* builder,
      drake::systems::rendering::PoseAggregator<double>* aggregator,
      drake::automotive::CarVisApplicator<double>* car_vis_applicator) override;

  int Initialize(drake::systems::Context<double>* context) override;

  drake::systems::System<double>* get_system() const;

 private:
  typedef drake::automotive::SimpleCarState<double> SimpleCarState;
  typedef std::unique_ptr<SimpleCarState> SimpleCarStatePtr;
  SimpleCarStatePtr simple_car_state_;
  drake::automotive::SimpleCar2<double>* simple_car_system_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
