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

#include "agents/rule_rail_car.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <drake/common/eigen_types.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/segment.h>

// public headers
#include "delphyne/macros.h"
#include "delphyne/roads/find_lane.h"
#include "gen/rail_follower_params.h"
#include "gen/rail_follower_state.h"

// private headers
#include "systems/lane_direction.h"
#include "systems/rail_follower.h"
#include "systems/right_of_way_system.h"
#include "systems/speed_system.h"
#include "systems/vector_source.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

RuleRailCarBlueprint::RuleRailCarBlueprint(const std::string& name, const maliput::api::Lane& lane,
                                           bool direction_of_travel,
                                           double longitudinal_position,  // s
                                           double lateral_offset,         // r
                                           double speed, double nominal_speed)
    : TypedAgentBlueprint<RuleRailCar>(name),
      initial_parameters_(lane, direction_of_travel, longitudinal_position, lateral_offset, speed, nominal_speed) {
  /*********************
   * Checks
   *********************/
  DELPHYNE_VALIDATE(initial_parameters_.lane.segment(), std::invalid_argument,
                    "The lane to be initialised on is not part of a road segment "
                    "(subsequently road geometry).");
  DELPHYNE_VALIDATE(initial_parameters_.lane.segment()->junction(), std::invalid_argument,
                    "The lane to be initialised on is not connected to a junction "
                    "(subsequently road geometry).");
  DELPHYNE_VALIDATE(initial_parameters_.lane.segment()->junction()->road_geometry(), std::invalid_argument,
                    "The lane to be initialised on is not part of a road geometry.");

  const maliput::api::LanePosition initial_car_lane_position{longitudinal_position, lateral_offset, 0.0};
  lane.ToInertialPosition(initial_car_lane_position);
  lane.GetOrientation(initial_car_lane_position);
}

std::unique_ptr<RuleRailCar> RuleRailCarBlueprint::DoBuildAgentInto(
    maliput::api::RoadNetwork* road_network, drake::systems::DiagramBuilder<double>* builder) const {
  DELPHYNE_VALIDATE(road_network != nullptr && road_network->road_geometry() != nullptr, std::invalid_argument,
                    "Rail cars need a road geometry to drive on, make "
                    "sure the simulation is built with one.");
  const maliput::api::RoadGeometry* lane_road_geometry =
      initial_parameters_.lane.segment()->junction()->road_geometry();
  DELPHYNE_VALIDATE(lane_road_geometry->id() == road_network->road_geometry()->id(), std::invalid_argument,
                    "The provided initial lane is not on the same road geometry "
                    "as that used by the simulation");
  DELPHYNE_VALIDATE(roads::FindLane(initial_parameters_.lane.id(), *road_network->road_geometry()),
                    std::invalid_argument,
                    "The provided initial lane is not within this simulation's "
                    "RoadGeometry.");

  /******************************************
   * Initial Context Variables
   ******************************************/
  typedef RailFollowerState<double> ContextContinuousState;
  typedef RailFollowerParams<double> ContextNumericParameters;

  ContextContinuousState context_continuous_state;
  context_continuous_state.set_s(initial_parameters_.position);
  context_continuous_state.set_speed(initial_parameters_.speed);
  ContextNumericParameters context_numeric_parameters;
  context_numeric_parameters.set_r(initial_parameters_.offset);
  context_numeric_parameters.set_h(0.0);
  context_numeric_parameters.set_max_speed(initial_parameters_.nominal_speed);

  /*********************
   * Instantiate System
   *********************/
  AgentBlueprint::DiagramBuilder agent_diagram_builder(this->name());

  // TODO(daniel.stonier): LaneDirection is discombabulating, it is more than
  // just direction and not even expressive of the lane's direction. Rather
  // it is intended to be used by the car (not it is not maliput api) to
  // indicate where in the road network it is and whether it is desired to
  // travel against the flow the lane's nominal direction (traffic flow).
  // Probably preferable to not use this at all and specify things separately.
  LaneDirection lane_direction(&(initial_parameters_.lane), initial_parameters_.direction_of_travel);
  RailFollower<double>* rail_follower_system = agent_diagram_builder.AddSystem(
      std::make_unique<RailFollower<double>>(lane_direction, context_continuous_state, context_numeric_parameters));
  rail_follower_system->set_name(name() + "_system");

  auto speed_setter = agent_diagram_builder.template AddSystem(
      std::make_unique<delphyne::VectorSource<double>>(initial_parameters_.speed));

  auto row_system = agent_diagram_builder.template AddSystem(std::make_unique<RightOfWaySystem<double>>(road_network));

  SpeedSystem<double>* speed_system = agent_diagram_builder.AddSystem<SpeedSystem>();

  agent_diagram_builder.Connect(speed_setter->output(), row_system->velocity_input());

  agent_diagram_builder.Connect(row_system->velocity_output(), speed_system->command_input());

  agent_diagram_builder.Connect(speed_system->acceleration_output(), rail_follower_system->command_input());

  agent_diagram_builder.Connect(rail_follower_system->velocity_output(), speed_system->feedback_input());

  agent_diagram_builder.Connect(rail_follower_system->lane_state_output(), row_system->lane_state_input());

  agent_diagram_builder.Connect(rail_follower_system->pose_output(), row_system->pose_input());

  /*********************
   * Diagram Outputs
   *********************/
  agent_diagram_builder.ExportStateOutput(rail_follower_system->simple_car_state_output());
  agent_diagram_builder.ExportPoseOutput(rail_follower_system->pose_output());
  agent_diagram_builder.ExportVelocityOutput(rail_follower_system->velocity_output());

  Agent::Diagram* agent_diagram = builder->AddSystem(agent_diagram_builder.Build());
  return std::make_unique<RuleRailCar>(agent_diagram, speed_setter);
}

RuleRailCar::RuleRailCar(Agent::Diagram* diagram, VectorSource<double>* speed_setter)
    : Agent(diagram), speed_setter_(speed_setter) {}

void RuleRailCar::SetSpeed(double new_speed_mps) { speed_setter_->Set(new_speed_mps); }

}  // namespace delphyne
