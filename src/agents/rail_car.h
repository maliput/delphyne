// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*****************************************************************************
** Includes
*****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <drake/systems/primitives/constant_vector_source.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_network.h>

// public headers
#include "delphyne/macros.h"
#include "delphyne/mi6/agent_base.h"
#include "delphyne/mi6/agent_base_blueprint.h"
#include "gen/maliput_railcar_state.h"
#include "systems/speed_system.h"
#include "systems/vector_source.h"

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
class RailCar : public Agent {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(RailCar)

  /// Rail car constructor.
  ///
  /// @param diagram The Diagram representation of the rail car agent.
  /// @param speed_setter The speed setting system associated to the
  ///                     rail car agent.
  explicit RailCar(Agent::Diagram* diagram, VectorSource<double>* speed_setter);

  /// Sets the speed of this agent.
  ///
  /// @param new_speed_mps The new speed for the agent,
  ///                      in meters per second.
  void SetSpeed(double new_speed_mps);

 private:
  // A source to set car speed, in meters per second.
  VectorSource<double>* speed_setter_;
};

/// @brief An agent that follows roads as if they were railroad tracks.
///
/// The underlying road network has a reference line for each lane which
/// is utilised by this agent as a railroad track.
///
/// Initial position is specified in the lane longitudinal co-ordinate (how far
/// along the track) and the agent will follow this track exactly - the only
/// variance it is permitted is the speed with which it follows the track.
class RailCarBlueprint : public TypedAgentBlueprint<RailCar> {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(RailCarBlueprint)

  /// Constructs a blueprint to build a RailCar into a simulation.
  ///
  /// @param name Unique name for the agent.
  /// @param lane The lane to start in.
  /// @param direction_of_travel Designates whether the car will travel
  ///            with or against the flow specified by the lane's rules
  /// @param longitudinal_position Initial position on the lane's track
  ///            (maliput lane coordinate, 's' (m)).
  /// @param lateral_offset The offset perpendicular to the centre road
  ///            line (maliput lane coordinate 'r' (m)).
  /// @param speed The actual initial speed.
  /// @param nominal_speed The desired cruising speed.
  explicit RailCarBlueprint(const std::string& name, const maliput::api::Lane& lane, bool direction_of_travel,
                            double longitudinal_position,  // s
                            double lateral_offset,         // r
                            double speed, double nominal_speed);

 private:
  // Container for the agent's initial configuration.
  //
  // Note: this is independent of whatever computational mechanisms
  // are used internally and is a useful construct for recording and
  // logging / streaming to debug configuration errors.
  struct Parameters {
    const maliput::api::Lane& lane;
    bool direction_of_travel{true};  // with or against the lane s-axis
    double position{0.0};            // longitudinal position in lane (m)
    double offset{0.0};              // lateral position in lane (m)
    double speed{0.0};               // speed in direction of the lane s-axis (m/s)
    double nominal_speed{0.0};       // nominal cruising speed (m/s)
    Parameters(const maliput::api::Lane& lane, bool direction_of_travel,
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

  std::unique_ptr<RailCar> DoBuildAgentInto(maliput::api::RoadNetwork* road_network,
                                            drake::systems::DiagramBuilder<double>* builder) const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
