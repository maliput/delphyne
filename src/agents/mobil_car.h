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

#include <maliput/api/road_geometry.h>
#include <drake/common/drake_copyable.h>

// public headers
#include "delphyne/mi6/agent_base_blueprint.h"

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
class MobilCarBlueprint : public BasicAgentBlueprint {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilCarBlueprint)

  /// @brief Default constructor
  ///
  /// @param name[in] The unique name for the agent
  /// @param direction_of_travel[in] Designates whether the car will travel
  ///            with or against the flow specified by the lane's rules.
  /// @param x[in] The scene x-coordinate.
  /// @param y[in] The scene y-coordinate.
  /// @param heading[in] The orientation of the car in the x-y frame.
  /// @param speed[in] The actual initial speed.
  explicit MobilCarBlueprint(const std::string& name, bool direction_of_travel,
                             double x, double y, double heading, double speed);

 private:
  struct Parameters {
    bool direction_of_travel{true};
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double offset{0.0};
    double speed{0.0};
    Parameters(bool direction_of_travel, double x, double y, double heading,
               double speed)
        : direction_of_travel(direction_of_travel),
          x(x),
          y(y),
          heading(heading),
          speed(speed) {}
  } initial_parameters_;

  std::unique_ptr<Agent::Diagram> DoBuildDiagram(
      const maliput::api::RoadGeometry* road_geometry) const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
