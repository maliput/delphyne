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

#include <chrono>
#include <cstdint>

#include "backend/ign_to_lcm_translation.h"
#include "drake/lcmt_driving_command_t.hpp"
#include <drake/lcmt_viewer_draw.hpp>
#include <ignition/msgs.hh>
#include "protobuf/automotive_driving_command.pb.h"

#include "backend/system.h"

namespace delphyne {
namespace backend {

int64_t millisFromSecs(int64_t secs) { return secs * 1000; }

int64_t millisFromNsecs(int64_t nsecs) { return nsecs / 1000000; }

std::vector<float> ignToVector(const ignition::msgs::Vector3d& position) {
  return {static_cast<float>(position.x()), static_cast<float>(position.y()),
          static_cast<float>(position.z())};
}

std::vector<float> ignToVector(const ignition::msgs::Quaternion& orientation) {
  return {
      static_cast<float>(orientation.w()), static_cast<float>(orientation.x()),
      static_cast<float>(orientation.y()), static_cast<float>(orientation.z())};
}

void ignToLcm(const ignition::msgs::AutomotiveDrivingCommand& ignDrivingCommand,
              drake::lcmt_driving_command_t* lcmDrivingCommand) {
  if (ignDrivingCommand.has_time()) {
    lcmDrivingCommand->timestamp =
        millisFromSecs(ignDrivingCommand.time().sec()) +
        millisFromNsecs(ignDrivingCommand.time().nsec());
  } else {
    int64_t milliseconds = std::chrono::system_clock::now().time_since_epoch() /
                           std::chrono::milliseconds(1);
    lcmDrivingCommand->timestamp = milliseconds;
  }
  lcmDrivingCommand->steering_angle = ignDrivingCommand.theta();
  lcmDrivingCommand->acceleration = ignDrivingCommand.acceleration();
}

void ignToLcm(const ignition::msgs::Model_V& robotModels,
              drake::lcmt_viewer_draw* robotDrawData) {
  robotDrawData->timestamp =
      millisFromSecs(robotModels.header().stamp().sec()) +
      millisFromNsecs(robotModels.header().stamp().nsec());

  // Each ignition model has many links using the same id, but this is
  // flattened in LCM
  robotDrawData->num_links = 0;
  for (int i = 0; i < robotModels.models_size(); ++i) {
    auto robotModel = robotModels.models(i);

    for (int j = 0; j < robotModel.link_size(); ++j) {
      robotDrawData->num_links += 1;
      auto link = robotModel.link(j);
      auto pose = link.pose();

      robotDrawData->robot_num.push_back(robotModel.id());
      robotDrawData->link_name.push_back(link.name());

      robotDrawData->position.push_back(ignToVector(pose.position()));
      robotDrawData->quaternion.push_back(ignToVector(pose.orientation()));
    }
  }
}

}  // namespace backend
}  // namespace delphyne
