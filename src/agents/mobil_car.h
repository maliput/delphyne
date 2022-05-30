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

#include <drake/common/drake_copyable.h>
#include <maliput/api/road_network.h>

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
  /// @param[in] name The unique name for the agent
  /// @param[in] direction_of_travel Designates whether the car will travel
  ///            with or against the flow specified by the lane's rules.
  /// @param[in] x The scene x-coordinate.
  /// @param[in] y The scene y-coordinate.
  /// @param[in] heading The orientation of the car in the x-y frame.
  /// @param[in] speed The actual initial speed.
  explicit MobilCarBlueprint(const std::string& name, bool direction_of_travel, double x, double y, double heading,
                             double speed);

 private:
  struct Parameters {
    bool direction_of_travel{true};
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double offset{0.0};
    double speed{0.0};
    Parameters(bool direction_of_travel, double x, double y, double heading, double speed)
        : direction_of_travel(direction_of_travel), x(x), y(y), heading(heading), speed(speed) {}
  } initial_parameters_;

  std::unique_ptr<Agent::Diagram> DoBuildDiagram(maliput::api::RoadNetwork* road_network) const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
