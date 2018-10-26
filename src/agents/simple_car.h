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

#include "delphyne/mi6/agent_base.h"
#include "gen/simple_car_state.h"
#include "systems/simple_car.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// @brief A very simple vehicle agent that can be teleoperated.
class SimpleCar : public Agent {
 public:
  SimpleCar(const std::string& name, double x, double y, double heading,
            double speed);

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

  std::unique_ptr<Diagram> BuildDiagram() const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
