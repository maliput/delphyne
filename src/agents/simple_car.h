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
#include <drake/geometry/scene_graph.h>
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

/// @brief A very simple vehicle agent that can be teleoperated.
class SimpleCar : public delphyne::Agent {
 public:
  SimpleCar(const std::string& name, double x, double y, double heading,
            double speed);
  void Configure(
      int id, const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<double>* builder,
      drake::geometry::SceneGraph<double>* scene_graph,
      drake::systems::rendering::PoseAggregator<double>* aggregator,
      drake::automotive::CarVisApplicator<double>* car_vis_applicator) override;

  void Print();

 private:
  // Container for the agent's initial configuration.
  //
  // Note: this is independent of whatever computational mechanisms
  // are used internally and is a useful construct for recording and
  // logging / streaming to debug configuration errors.
  struct Parameters {
    double x{0.0};        // scene x-coordinate (m)
    double y{0.0};        // scene y-coordinate (m)
    double heading{0.0};  // scene heading (around z-axis (radians)
    double speed{0.0};    // speed in axis defined by the heading (m/s)
    Parameters(double x, double y, double heading, double speed)
        : x(x), y(y), heading(heading), speed(speed) {}
  } initial_parameters_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
