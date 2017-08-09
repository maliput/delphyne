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

#include <ignition/common/Console.hh>

#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "lcm_channel_repeater.hh"

#include "lcm/lcm-cpp.hpp"

// Runs a program that listens for new messages in a set
// of Drake LCM channels and converts them to ign-messages
// to be consumed by the front end.
int main(int argc, char* argv[]) {
  lcm::LCM lcm;

  ignition::common::Console::SetVerbosity(3);
  ignmsg << "LCM to ignition-transport bridge 0.1.0" << std::endl;

  try {
    // Create a repeater on DRAKE_VIEWER_LOAD_ROBOT channel, translating
    // from drake::lcmt_viewer_load_robot to ignition::msgs::Model
    delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_load_robot,
                                         ignition::msgs::Model>
        viewerLoadRobotRepeater(lcm, "DRAKE_VIEWER_LOAD_ROBOT");

    viewerLoadRobotRepeater.Start();
  } catch(const std::runtime_error &error) {
    ignerr << "Failed to start LCM channel repeater for initialize DRAKE_VIEWER_LOAD_ROBOT" << std::endl;
    ignerr << "Details: " << error.what() << std::endl;
    exit(1);
  }

  while (true) {
    lcm.handle();
  }

  return 0;
}
