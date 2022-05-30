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

#include <maliput/api/road_network.h>

#include "delphyne/mi6/agent_base_blueprint.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// @brief A very simple vehicle agent that can be teleoperated.
class SimpleCarBlueprint : public BasicAgentBlueprint {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleCarBlueprint)

  explicit SimpleCarBlueprint(const std::string& name, double x, double y, double heading, double speed);

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
    Parameters(double x, double y, double heading, double speed) : x(x), y(y), heading(heading), speed(speed) {}
  } initial_parameters_;

  std::unique_ptr<Agent::Diagram> DoBuildDiagram(maliput::api::RoadNetwork* road_network) const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
