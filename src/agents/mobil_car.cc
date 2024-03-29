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

#include "agents/mobil_car.h"

#include <memory>
#include <string>
#include <utility>

#include <drake/common/eigen_types.h>
#include <drake/systems/primitives/multiplexer.h>
#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

// public headers
#include "delphyne/macros.h"

// private headers
#include "systems/idm_controller.h"
#include "systems/mobil_planner.h"
#include "systems/pure_pursuit_controller.h"
#include "systems/simple_car.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

MobilCarBlueprint::MobilCarBlueprint(const std::string& name, bool direction_of_travel, double x, double y,
                                     double heading, double speed)
    : BasicAgentBlueprint(name), initial_parameters_(direction_of_travel, x, y, heading, speed) {}

std::unique_ptr<Agent::Diagram> MobilCarBlueprint::DoBuildDiagram(maliput::api::RoadNetwork* road_network) const {
  DELPHYNE_VALIDATE(road_network != nullptr && road_network->road_geometry() != nullptr, std::invalid_argument,
                    "MOBIL cars require a road geometry to run, make "
                    "sure the simulation was setup with one.");
  AgentBlueprint::DiagramBuilder builder(this->name());

  /******************************************
   * Initial Context Variables
   ******************************************/
  typedef delphyne::SimpleCarState<double> ContextContinuousState;
  typedef delphyne::SimpleCarParams<double> ContextNumericParameters;
  ContextContinuousState context_continuous_state;
  context_continuous_state.set_x(initial_parameters_.x);
  context_continuous_state.set_y(initial_parameters_.y);
  context_continuous_state.set_heading(initial_parameters_.heading);
  context_continuous_state.set_velocity(initial_parameters_.speed);
  ContextNumericParameters context_numeric_parameters;

  /*********************
   * Instantiate Systems
   *********************/
  delphyne::MobilPlanner<double>* mobil_planner = builder.AddSystem(std::make_unique<delphyne::MobilPlanner<double>>(
      *road_network->road_geometry(), initial_parameters_.direction_of_travel, RoadPositionStrategy::kExhaustiveSearch,
      0. /* time period (unused) */));
  mobil_planner->set_name(this->name() + "_mobil_planner");

  IDMController<double>* idm_controller = builder.AddSystem(
      std::make_unique<IDMController<double>>(*road_network->road_geometry(), ScanStrategy::kBranches,
                                              RoadPositionStrategy::kExhaustiveSearch, 0. /* time period (unused) */));
  idm_controller->set_name(this->name() + "_idm_controller");

  delphyne::PurePursuitController<double>* pursuit =
      builder.AddSystem(std::make_unique<delphyne::PurePursuitController<double>>());
  pursuit->set_name(this->name() + "_pure_pursuit_controller");

  typedef SimpleCar2<double> SimpleCarSystem;
  SimpleCarSystem* simple_car_system =
      builder.AddSystem(std::make_unique<SimpleCarSystem>(context_continuous_state, context_numeric_parameters));
  simple_car_system->set_name(this->name() + "_simple_car");

  drake::systems::Multiplexer<double>* mux = builder.AddSystem<drake::systems::Multiplexer<double>>(
      std::make_unique<drake::systems::Multiplexer<double>>(DrivingCommand<double>()));
  mux->set_name(this->name() + "_mux");

  /*********************
   * Diagram Wiring
   *********************/
  // driving
  builder.Connect(simple_car_system->pose_output(), mobil_planner->ego_pose_input());
  builder.Connect(simple_car_system->velocity_output(), mobil_planner->ego_velocity_input());
  builder.Connect(idm_controller->acceleration_output(), mobil_planner->ego_acceleration_input());
  builder.Connect(simple_car_system->pose_output(), idm_controller->ego_pose_input());
  builder.Connect(simple_car_system->velocity_output(), idm_controller->ego_velocity_input());

  builder.ConnectTrafficPosesTo(mobil_planner->traffic_input());
  builder.ConnectTrafficPosesTo(idm_controller->traffic_input());

  builder.Connect(simple_car_system->pose_output(), pursuit->ego_pose_input());
  builder.Connect(mobil_planner->lane_output(), pursuit->lane_input());
  // Build DrivingCommand via a mux of two scalar outputs (a BasicVector where
  // row 0 = steering command, row 1 = acceleration command).
  builder.Connect(pursuit->steering_command_output(), mux->get_input_port(0));
  builder.Connect(idm_controller->acceleration_output(), mux->get_input_port(1));
  builder.Connect(mux->get_output_port(0), simple_car_system->get_input_port(0));

  /*********************
   * Diagram Outputs
   *********************/
  builder.ExportStateOutput(simple_car_system->state_output());
  builder.ExportPoseOutput(simple_car_system->pose_output());
  builder.ExportVelocityOutput(simple_car_system->velocity_output());

  return builder.Build();
}

}  // namespace delphyne
