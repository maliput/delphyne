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

#include "agents/simple_car.h"

#include <string>
#include <utility>

#include <maliput/common/maliput_unused.h>

#include "backend/ign_subscriber_system.h"
#include "gen/simple_car_state.h"
#include "systems/simple_car.h"
#include "translations/ign_driving_command_to_drake.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

SimpleCarBlueprint::SimpleCarBlueprint(const std::string& name, double x, double y, double heading, double speed)
    : BasicAgentBlueprint(name), initial_parameters_(x, y, heading, speed) {}

std::unique_ptr<Agent::Diagram> SimpleCarBlueprint::DoBuildDiagram(maliput::api::RoadNetwork* road_network) const {
  maliput::common::unused(road_network);

  AgentBlueprint::DiagramBuilder builder(this->name());

  /*********************
   * Context
   *********************/
  typedef SimpleCarState<double> ContextContinuousState;
  typedef SimpleCarParams<double> ContextNumericParameters;
  ContextContinuousState context_continuous_state;
  context_continuous_state.set_x(initial_parameters_.x);
  context_continuous_state.set_y(initial_parameters_.y);
  context_continuous_state.set_heading(initial_parameters_.heading);
  context_continuous_state.set_velocity(initial_parameters_.speed);
  ContextNumericParameters context_numeric_parameters;

  /*********************
   * Simple Car System
   *********************/
  typedef SimpleCar2<double> SimpleCarSystem;
  SimpleCarSystem* simple_car_system =
      builder.AddSystem(std::make_unique<SimpleCarSystem>(context_continuous_state, context_numeric_parameters));
  simple_car_system->set_name(this->name() + "_system");

  /*********************
   * Teleop Systems
   *********************/
  std::string command_channel = "teleop/" + this->name();
  typedef IgnSubscriberSystem<ignition::msgs::AutomotiveDrivingCommand> DrivingCommandSubscriber;
  DrivingCommandSubscriber* driving_command_subscriber =
      builder.AddSystem(std::make_unique<DrivingCommandSubscriber>(command_channel));

  auto driving_command_translator = builder.AddSystem(std::make_unique<IgnDrivingCommandToDrake>());

  // Ignition driving commands received through the subscriber are translated
  // to Drake.
  builder.Connect(*driving_command_subscriber, *driving_command_translator);

  // And then the translated Drake command is sent to the car.
  builder.Connect(*driving_command_translator, *simple_car_system);

  /*********************
   * Diagram Outputs
   *********************/
  builder.ExportStateOutput(simple_car_system->state_output());
  builder.ExportPoseOutput(simple_car_system->pose_output());
  builder.ExportVelocityOutput(simple_car_system->velocity_output());

  return builder.Build();
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
