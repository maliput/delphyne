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

#include <iostream>
#include <thread>
#include <lcm/lcm-cpp.hpp>

#include <ignition/common/Console.hh>
#include "lcmtypes/lcmt_viewer_geometry_data.hpp"
#include "lcmtypes/lcmt_viewer_link_data.hpp"
#include "lcmtypes/lcmt_viewer_load_robot.hpp"

// Publishes a defined lcmt_viewer_load_robot
// message into the DRAKE_VIEWER_LOAD_ROBOT
// channel every one second
int main(int argc, char* argv[]) {
  lcm::LCM lcm;

  if (!lcm.good()) {
    ignerr << "Failed to initialize LCM" << std::endl;
    return 1;
  }

  // Generate geometry message
  drake::lcmt_viewer_geometry_data geometryMsg;
  geometryMsg.type = geometryMsg.BOX;
  geometryMsg.num_float_data = 3;
  geometryMsg.float_data.resize(geometryMsg.num_float_data);
  geometryMsg.float_data[0] = 3.8808;
  geometryMsg.float_data[1] = 0.75;
  geometryMsg.float_data[2] = 0.030921;

  // Generate link message
  drake::lcmt_viewer_link_data linkMsg;
  linkMsg.name = "chassis";
  linkMsg.robot_num = 0;
  linkMsg.num_geom = 1;
  linkMsg.geom.resize(linkMsg.num_geom);
  linkMsg.geom[0] = geometryMsg;

  // Generate robot message
  drake::lcmt_viewer_load_robot robotMsg;
  robotMsg.num_links = 1;
  robotMsg.link.resize(robotMsg.num_links);
  robotMsg.link[0] = linkMsg;

  // Publish robot messages at 1Hz.
  while (1) {
    lcm.publish("DRAKE_VIEWER_LOAD_ROBOT", &robotMsg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
