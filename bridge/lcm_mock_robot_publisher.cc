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
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/math/Helpers.hh>

#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_link_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
static void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) terminatePub = true;
}

//////////////////////////////////////////////////
// \brief Publishes a defined lcmt_viewer_load_robot
// message into the DRAKE_VIEWER_LOAD_ROBOT channel.
// It consists on:
// - A box
// - A cylinder
// - A sphere
// - A mesh loaded from a URL path
// - A mesh loaded from a "package" styled path
int main(int argc, char* argv[]) {
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  ignition::common::Console::SetVerbosity(3);
  ignmsg << "Starting up mock publisher" << std::endl;

  lcm::LCM lcm;
  if (!lcm.good()) {
    ignerr << "Failed to initialize LCM" << std::endl;
    return 1;
  }
  // fixed values used to override the non-initialized variables from the
  // geometry messages
  std::vector<float> quaternion = {1.0, 0.0, 0.0, 0.0};
  std::vector<float> color = {1.0, 1.0, 1.0, 1.0};

  ////////////////
  /// BOX
  ////////////////

  // Define box geometry message
  drake::lcmt_viewer_geometry_data boxMsg;
  boxMsg.type = boxMsg.BOX;
  boxMsg.num_float_data = 3;
  boxMsg.float_data.resize(boxMsg.num_float_data);
  boxMsg.float_data[0] = 3.8808;
  boxMsg.float_data[1] = 0.75;
  boxMsg.float_data[2] = 0.030921;
  boxMsg.position[0] = 0.0;
  boxMsg.position[1] = 0.0;
  boxMsg.position[2] = 0.0;
  std::copy(quaternion.begin(), quaternion.end(), boxMsg.quaternion);
  std::copy(color.begin(), color.end(), boxMsg.color);

  // Generate box link message
  drake::lcmt_viewer_link_data boxLinkMsg;
  boxLinkMsg.name = "box";
  boxLinkMsg.robot_num = 1;
  boxLinkMsg.num_geom = 1;
  boxLinkMsg.geom.resize(boxLinkMsg.num_geom);
  boxLinkMsg.geom[0] = boxMsg;

  ////////////////
  /// SPHERE
  ////////////////

  // Define sphere geometry message
  drake::lcmt_viewer_geometry_data sphereMsg;
  sphereMsg.type = sphereMsg.SPHERE;
  sphereMsg.num_float_data = 1;
  sphereMsg.float_data.resize(sphereMsg.num_float_data);
  sphereMsg.float_data[0] = 1;
  sphereMsg.position[0] = 3;
  sphereMsg.position[1] = 3;
  sphereMsg.position[2] = 0.5;
  std::copy(quaternion.begin(), quaternion.end(), sphereMsg.quaternion);
  std::copy(color.begin(), color.end(), sphereMsg.color);

  // Generate sphere link message
  drake::lcmt_viewer_link_data sphereLinkMsg;
  sphereLinkMsg.name = "sphere";
  sphereLinkMsg.robot_num = 1;
  sphereLinkMsg.num_geom = 1;
  sphereLinkMsg.geom.resize(sphereLinkMsg.num_geom);
  sphereLinkMsg.geom[0] = sphereMsg;

  ////////////////
  /// CYLINDER
  ////////////////

  // Define cylinder geometry message
  drake::lcmt_viewer_geometry_data cylinderMsg;
  cylinderMsg.type = cylinderMsg.CYLINDER;
  cylinderMsg.num_float_data = 2;
  cylinderMsg.float_data.resize(cylinderMsg.num_float_data);
  cylinderMsg.float_data[0] = 1;
  cylinderMsg.float_data[1] = 4;
  cylinderMsg.position[0] = -3;
  cylinderMsg.position[1] = -3;
  cylinderMsg.position[2] = 2;
  std::copy(quaternion.begin(), quaternion.end(), cylinderMsg.quaternion);
  std::copy(color.begin(), color.end(), cylinderMsg.color);

  // Generate cylinder link message
  drake::lcmt_viewer_link_data cylinderLinkMsg;
  cylinderLinkMsg.name = "cylinder";
  cylinderLinkMsg.robot_num = 18;
  cylinderLinkMsg.num_geom = 1;
  cylinderLinkMsg.geom.resize(cylinderLinkMsg.num_geom);
  cylinderLinkMsg.geom[0] = cylinderMsg;

  ////////////////
  /// MESHES
  ////////////////

  // Define mesh-from-url geometry message
  drake::lcmt_viewer_geometry_data meshURLMsg;
  meshURLMsg.type = meshURLMsg.MESH;
  meshURLMsg.string_data =
      "media/duck.dae";  // Relative to DELPHYNE_PACKAGE_PATH
                         // or an absolute path.
  meshURLMsg.num_float_data = 3;
  meshURLMsg.float_data.resize(meshURLMsg.num_float_data);
  meshURLMsg.float_data[0] = 1.0;  // scale
  meshURLMsg.float_data[1] = 1.0;
  meshURLMsg.float_data[2] = 0.5;
  meshURLMsg.position[0] = 6;
  meshURLMsg.position[1] = 3;
  meshURLMsg.position[2] = 0;
  meshURLMsg.quaternion[0] = 0.707;
  meshURLMsg.quaternion[1] = 0.707;
  meshURLMsg.quaternion[2] = 0.0;
  meshURLMsg.quaternion[3] = 0.0;
  std::copy(color.begin(), color.end(), meshURLMsg.color);

  // Generate mesh-from-url link message
  drake::lcmt_viewer_link_data meshURLLinkMsg;
  meshURLLinkMsg.name = "mesh-from-url";
  meshURLLinkMsg.robot_num = 18;
  meshURLLinkMsg.num_geom = 1;
  meshURLLinkMsg.geom.resize(meshURLLinkMsg.num_geom);
  meshURLLinkMsg.geom[0] = meshURLMsg;

  // Define mesh-from-package geometry message
  drake::lcmt_viewer_geometry_data meshPackageMsg;
  meshPackageMsg.type = meshPackageMsg.MESH;
  // The following path gets solved by drake's resolvePackageFilename. Requires
  // a package.xml file on each named directory and to append the root dir to
  // DELPHYNE_PACKAGE_PATH, in this case, must be relative to drake's models dir
  meshPackageMsg.string_data = "package://media/duck.dae";
  meshPackageMsg.num_float_data = 3;
  meshPackageMsg.float_data.resize(meshPackageMsg.num_float_data);
  meshPackageMsg.float_data[0] = 1.0;  // scale
  meshPackageMsg.float_data[1] = 1.0;
  meshPackageMsg.float_data[2] = 0.5;
  meshPackageMsg.position[0] = -6;
  meshPackageMsg.position[1] = 3;
  meshPackageMsg.position[2] = 0;
  meshPackageMsg.quaternion[0] = 0.707;
  meshPackageMsg.quaternion[1] = 0.707;
  meshPackageMsg.quaternion[2] = 0.0;
  meshPackageMsg.quaternion[3] = 0.0;
  std::copy(color.begin(), color.end(), meshPackageMsg.color);

  // Generate mesh-from-package link message
  drake::lcmt_viewer_link_data meshPackageLinkMsg;
  meshPackageLinkMsg.name = "mesh-from-package";
  meshPackageLinkMsg.robot_num = 18;
  meshPackageLinkMsg.num_geom = 1;
  meshPackageLinkMsg.geom.resize(meshPackageLinkMsg.num_geom);
  meshPackageLinkMsg.geom[0] = meshPackageMsg;

  // Generate robot message
  drake::lcmt_viewer_load_robot robotMsg;
  robotMsg.num_links = 5;
  robotMsg.link.resize(robotMsg.num_links);
  robotMsg.link[0] = boxLinkMsg;
  robotMsg.link[1] = sphereLinkMsg;
  robotMsg.link[2] = cylinderLinkMsg;
  robotMsg.link[3] = meshURLLinkMsg;
  robotMsg.link[4] = meshPackageLinkMsg;

  // Define draw message
  drake::lcmt_viewer_draw drawMsg;
  drawMsg.timestamp = 0;
  drawMsg.num_links = 1;
  drawMsg.link_name.resize(drawMsg.num_links);
  drawMsg.link_name[0] = "box";
  drawMsg.robot_num.resize(1);
  drawMsg.robot_num[0] = 1;
  drawMsg.position.resize(1);
  drawMsg.position[0].resize(3);
  drawMsg.position[0][0] = 0.0;
  drawMsg.position[0][1] = 0.0;
  drawMsg.position[0][2] = 0.0;
  drawMsg.quaternion.resize(1);
  drawMsg.quaternion[0].resize(4);
  drawMsg.quaternion[0][0] = 0.0;
  drawMsg.quaternion[0][1] = 0.0;
  drawMsg.quaternion[0][2] = 0.0;
  drawMsg.quaternion[0][3] = 1.0;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Load robot model into drake
  std::string lcmChannel = "DRAKE_VIEWER_LOAD_ROBOT";
  igndbg << "Publishing message into " << lcmChannel << std::endl;
  if (lcm.publish(lcmChannel, &robotMsg) == -1) {
    ignerr << "Failed to publish message into " << lcmChannel << std::endl;
    return 1;
  }

  // Circumference definition
  float radius = 3.0;
  std::vector<float> center = {0.0, 0.0, 4.0};
  // Initialize timestep and counter
  int timeStepMs = 10;
  int i = 0;
  // Update sphere position every 10ms
  while (!terminatePub) {
    // Update timestamp
    drawMsg.timestamp = drawMsg.timestamp + timeStepMs;
    // Update each position by moving 1 degree along the previously defined
    // circumference
    drawMsg.position[0][0] =
        center[0] + radius * std::sin(i * 2 * IGN_PI / 360);
    drawMsg.position[0][2] =
        center[2] + radius * std::cos(i * 2 * IGN_PI / 360);
    // Publish message with updated position
    igndbg << "Publishing message into DRAKE_VIEWER_DRAW" << std::endl;
    if (lcm.publish("DRAKE_VIEWER_DRAW", &drawMsg) == -1) {
      ignerr << "Failed to publish message into DRAKE_VIEWER_DRAW" << std::endl;
      return 1;
    }
    // Wait 10ms
    std::this_thread::sleep_for(std::chrono::milliseconds(timeStepMs));
    // Increment i value and keep it bounded between 0 and 359
    i = (i + 1) % 360;
  }
  return 0;
}
