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
class UnicycleCarAgent : public Agent {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(UnicycleCarAgent)

  /// Unicycle car agent constructor.
  ///
  /// @param diagram The Diagram representation of the rail car agent.
  /// @param acceleration_setter The acceleration input to the underlying UnicycleCar
  /// in this diagram.
  /// @param angular_rate_setter The angular rate input to the underlying UnicycleCar
  /// in this diagram.

  explicit UnicycleCarAgent(Agent::Diagram* diagram, VectorSource<double>* acceleration_setter,
                            VectorSource<double>* angular_rate_setter)
      : Agent(diagram), acceleration_setter_(acceleration_setter), angular_rate_setter_(angular_rate_setter) {}

  /// Sets the acceleration input to this agent.
  ///
  /// @param new_acceleration The new acceleration input to the agent.
  void SetAcceleration(double new_acceleration) { acceleration_setter_->Set(new_acceleration); }

  /// Sets the angular rate input to this agent.
  ///
  /// @param new_angular_rate The new angular rate input to the agent.
  void SetAngularRate(double new_angular_rate) { angular_rate_setter_->Set(new_angular_rate); }

 private:
  VectorSource<double>* acceleration_setter_;
  VectorSource<double>* angular_rate_setter_;
};

/// @brief A very simple vehicle agent with settable inputs.
class UnicycleCarBlueprint : public TypedAgentBlueprint<UnicycleCarAgent> {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(UnicycleCarBlueprint)

  explicit UnicycleCarBlueprint(const std::string& name, double x, double y, double heading, double speed);

 private:
  // Container for the agent's initial configuration.
  //
  // Note: this is independent of whatever computational mechanisms
  // are used internally and is a useful construct for recording and
  // logging / streaming to debug configuration errors.
  struct InitialConditions {
    double x{0.0};        // scene x-coordinate (m)
    double y{0.0};        // scene y-coordinate (m)
    double heading{0.0};  // scene heading (around z-axis (radians)
    double speed{0.0};    // speed in axis defined by the heading (m/s)
    InitialConditions(double x, double y, double heading, double speed) : x(x), y(y), heading(heading), speed(speed) {}
  } initial_conditions_;

  std::unique_ptr<UnicycleCarAgent> DoBuildAgentInto(
      maliput::api::RoadNetwork* road_network,
      drake::systems::DiagramBuilder<double>* simulator_builder) const override;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
