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

// Drake LCM message headers
#include "drake/lcmt_driving_command_t.hpp"
#include "drake/lcmt_simple_car_state_t.hpp"
#include "drake/lcmt_viewer_command.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/viewer2_comms_t.hpp"

// Custom ignition message headers
#include "protobuf/headers/automotive_driving_command.pb.h"
#include "protobuf/headers/simple_car_state.pb.h"
#include "protobuf/headers/viewer_command.pb.h"
#include "protobuf/headers/viewer2_comms.pb.h"

// Repeater classes
#include "ign_service_converter.hh"
#include "ign_to_lcm_translation.hh"
#include "ign_topic_repeater.hh"
#include "lcm_channel_repeater.hh"
#include "repeater_factory.hh"
#include "repeater_manager.hh"
#include "service_to_channel_translation.hh"

// LCM entry point
#include "lcm/lcm-cpp.hpp"

// Register custom msg. Note that the name has to include "ign_msgs" at the
// beginning
IGN_REGISTER_STATIC_MSG("ign_msgs.AutomotiveDrivingCommand",
                        AutomotiveDrivingCommand)

// Register a topic repeater for AutomotiveDrivingCommand

REGISTER_STATIC_REPEATER("ign_msgs.AutomotiveDrivingCommand",
                         ignition::msgs::AutomotiveDrivingCommand,
                         drake::lcmt_driving_command_t, 1)

/// \brief Flag used to break the LCM loop and terminate the program.
static std::atomic<bool> terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that handles LCM and
/// exit the program smoothly.
static void signalHandler(int signal) {
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


  // Number of cars passed as argument defines the number of
  // simple_car_state translators.
  // This approach is temporal, and will be removed as soon as the
  // dynamic creation of lcm-to-ign repeaters is ready
  int numCars = 2;
  if (argc >= 2) {
    std::istringstream iss(argv[1]);
    int val;
    if (iss >> val) {
      numCars = val;
    }
  }

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

  // Create a repeater on DRAKE_VIEWER_LOAD_ROBOT channel, translating
  // from drake::lcmt_viewer_load_robot to ignition::msgs::Model
  delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_load_robot,
                                       ignition::msgs::Model_V>
      viewerLoadRobotRepeater(sharedLCM, "DRAKE_VIEWER_LOAD_ROBOT");

  // Create a repeater on DRAKE_VIEWER_DRAW channel, translating
  // from drake::lcmt_viewer_draw to ignition::msgs::PosesStamped
  delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_draw,
                                       ignition::msgs::Model_V>
      viewerDrawRepeater(sharedLCM, "DRAKE_VIEWER_DRAW");

  // Create a repeater on DRAKE_VIEWER_STATUS channel, translating
  // from drake::lcmt_viewer_command to ignition::msgs::ViewerCommand
  delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_command,
                                       ignition::msgs::ViewerCommand>
      viewerCommandRepeater(sharedLCM, "DRAKE_VIEWER_STATUS");

  // Create a vector of repeaters on X_SIMPLE_CAR_STATE translating
  // from drake::lcmt_simple_car_state_t to ignition::msgs::SimpleCarState
  typedef delphyne::bridge::LcmChannelRepeater<drake::lcmt_simple_car_state_t,
                                               ignition::msgs::SimpleCarState>
      simpleCarRepeater_t;
  std::vector<std::shared_ptr<simpleCarRepeater_t> > simpleCarRepeaterVector;
  for (int i = 0; i < numCars; i++) {
    simpleCarRepeaterVector.push_back(std::make_shared<simpleCarRepeater_t>(
        sharedLCM, std::to_string(i) + "_SIMPLE_CAR_STATE"));
  }

  // Create a repeater on DIRECTOR_TREE_VIEWER_RESPONSE channel, translating
  // from drake::viewer2_comms to ignition::msgs::Viewer2Comms
  delphyne::bridge::LcmChannelRepeater<drake::viewer2_comms_t,
                                       ignition::msgs::Viewer2Comms>
      viewer2CommsRepeater(sharedLCM, "DIRECTOR_TREE_VIEWER_RESPONSE");

  // Start DRAKE_VIEWER_LOAD_ROBOT repeater
  try {
    viewerLoadRobotRepeater.Start();
  } catch (const std::runtime_error& error) {
    ignerr << "Failed to start LCM channel repeater for "
           << "DRAKE_VIEWER_LOAD_ROBOT" << std::endl;
    ignerr << "Details: " << error.what() << std::endl;
    exit(1);
  }
  // Start DRAKE_VIEWER_DRAW repeater
  try {
    viewerDrawRepeater.Start();
  } catch (const std::runtime_error& error) {
    ignerr << "Failed to start LCM channel repeater for "
           << "DRAKE_VIEWER_DRAW" << std::endl;
    ignerr << "Details: " << error.what() << std::endl;
    exit(1);
  }
  // Start DRAKE_VIEWER_STATUS repeater
  try {
    viewerCommandRepeater.Start();
  } catch (const std::runtime_error& error) {
    ignerr << "Failed to start LCM channel repeater for "
           << "DRAKE_VIEWER_STATUS" << std::endl;
    ignerr << "Details: " << error.what() << std::endl;
    exit(1);
  }
  // Start all the X_SIMPLE_CAR_STATE repeaters
  for (auto i = 0u; i < simpleCarRepeaterVector.size(); i++) {
    try {
      simpleCarRepeaterVector[i]->Start();
    } catch (const std::runtime_error& error) {
      ignerr << "Failed to start LCM channel repeater for "
             << std::to_string(i) + "_SIMPLE_CAR_STATE" << std::endl;
      ignerr << "Details: " << error.what() << std::endl;
      exit(1);
    }
  }
  // Start DIRECTOR_TREE_VIEWER_RESPONSE repeater
  try {
    viewer2CommsRepeater.Start();
  } catch (const std::runtime_error& error) {
    ignerr << "Failed to start LCM channel repeater for "
           << "DIRECTOR_TREE_VIEWER_RESPONSE" << std::endl;
    ignerr << "Details: " << error.what() << std::endl;
    exit(1);
  }

  // Service name
  std::string notifierServiceName = "/visualizer_start_notifier";
  std::string channelName = "DRAKE_VIEWER_STATUS";

  // Start ignition service to lcm channel converter
  delphyne::bridge::IgnitionServiceConverter<ignition::msgs::Empty,
                                             drake::lcmt_viewer_command>
      ignToLcmRepublisher(sharedLCM, notifierServiceName, channelName);
  ignToLcmRepublisher.Start();

  while (!terminatePub) {
    sharedLCM->handleTimeout(100);
  }

  return 0;
}
