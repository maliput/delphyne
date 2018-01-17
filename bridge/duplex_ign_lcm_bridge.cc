// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <atomic>
#include <csignal>
#include <sstream>
#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>

// LCM entry point
#include <lcm/lcm-cpp.hpp>

// Drake LCM message headers
#include <drake/lcmt_driving_command_t.hpp>
#include <drake/lcmt_simple_car_state_t.hpp>
#include <drake/lcmt_viewer_command.hpp>
#include <drake/lcmt_viewer_draw.hpp>
#include <drake/lcmt_viewer_geometry_data.hpp>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <robotlocomotion/viewer2_comms_t.hpp>

// Custom ignition message headers
#include <protobuf/automotive_driving_command.pb.h>
#include <protobuf/simple_car_state.pb.h>
#include <protobuf/viewer2_comms.pb.h>
#include <protobuf/viewer_command.pb.h>

// Repeater classes
#include "bridge/ign_service_converter.h"
#include "bridge/ign_to_lcm_translation.h"
#include "bridge/ign_topic_repeater.h"
#include "bridge/lcm_channel_repeater.h"
#include "bridge/repeater_factory.h"
#include "bridge/repeater_manager.h"
#include "bridge/service_to_channel_translation.h"

// Register custom msg. Note that the name has to include "ign_msgs" at the
// beginning
IGN_REGISTER_STATIC_MSG("ign_msgs.AutomotiveDrivingCommand",
                        AutomotiveDrivingCommand)

// Register a repeater translating from ignition::msgs::AutomotiveDrivingCommand
// to drake::lcmt_driving_command_t
REGISTER_STATIC_IGN_REPEATER("DRIVING_COMMAND_(.*)",
                             ignition::msgs::AutomotiveDrivingCommand,
                             drake::lcmt_driving_command_t)

REGISTER_STATIC_LCM_REPEATER("(.*)_SIMPLE_CAR_STATE",
                             drake::lcmt_simple_car_state_t,
                             ignition::msgs::SimpleCarState)

/// \brief Flag used to break the LCM loop and terminate the program.
static std::atomic<bool> terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that handles LCM and
/// exit the program smoothly.
void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    terminatePub = true;
  }
}

// Runs a program that listens for new messages in a set
// of Drake LCM channels and converts them to ign-messages
// to be consumed by the front end.
int main(int argc, char* argv[]) {
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  ignition::common::Console::SetVerbosity(3);
  ignmsg << "LCM to ignition-transport bridge 0.1.0" << std::endl;

  std::shared_ptr<lcm::LCM> sharedLCM = std::make_shared<lcm::LCM>();

  delphyne::bridge::RepeaterManager manager(sharedLCM);

  try {
    manager.Start();
  } catch (const std::runtime_error& error) {
    ignerr << "Failed to start the repeater manager" << std::endl;
    ignerr << "Details: " << error.what() << std::endl;
    exit(1);
  }

  while (!terminatePub) {
    sharedLCM->handleTimeout(100);
  }

  return 0;
}
