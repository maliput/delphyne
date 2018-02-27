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

#include "backend/ign_to_lcm_translation.h"

#include <chrono>
#include <cstdint>
#include <vector>

#include "drake/lcmt_driving_command_t.hpp"
#include "drake/lcmt_viewer_draw.hpp"

#include "ignition/msgs.hh"

#include "protobuf/automotive_driving_command.pb.h"

#include "backend/system.h"
#include "backend/time_conversion.h"

namespace delphyne {
namespace backend {

std::vector<float> ignToVector(const ignition::msgs::Vector3d& position) {
  return {static_cast<float>(position.x()), static_cast<float>(position.y()),
          static_cast<float>(position.z())};
}

std::vector<float> ignToVector(const ignition::msgs::Quaternion& orientation) {
  return {
      static_cast<float>(orientation.w()), static_cast<float>(orientation.x()),
      static_cast<float>(orientation.y()), static_cast<float>(orientation.z())};
}

void ignToLcm(
    const ignition::msgs::AutomotiveDrivingCommand& ign_driving_command,
    drake::lcmt_driving_command_t* lcm_driving_command) {
  if (ign_driving_command.has_time()) {
    lcm_driving_command->timestamp =
        IgnitionTimeToMillis(ign_driving_command.time());
  } else {
    int64_t milliseconds = std::chrono::system_clock::now().time_since_epoch() /
                           std::chrono::milliseconds(1);
    lcm_driving_command->timestamp = milliseconds;
  }
  lcm_driving_command->steering_angle = ign_driving_command.theta();
  lcm_driving_command->acceleration = ign_driving_command.acceleration();
}

void ignToLcm(const ignition::msgs::Model_V& robot_models,
              drake::lcmt_viewer_draw* robot_draw_data) {
  robot_draw_data->timestamp =
      IgnitionTimeToMillis(robot_models.header().stamp());

  // Each ignition model has many links using the same id, but this is
  // flattened in LCM
  robot_draw_data->num_links = 0;
  for (int i = 0; i < robot_models.models_size(); ++i) {
    const ::ignition::msgs::Model& robot_model = robot_models.models(i);

    for (int j = 0; j < robot_model.link_size(); ++j) {
      const ::ignition::msgs::Link& link = robot_model.link(j);
      const ::ignition::msgs::Pose& pose = link.pose();

      robot_draw_data->robot_num.push_back(robot_model.id());
      robot_draw_data->link_name.push_back(link.name());

      robot_draw_data->position.push_back(ignToVector(pose.position()));
      robot_draw_data->quaternion.push_back(ignToVector(pose.orientation()));

      robot_draw_data->num_links += 1;
    }
  }
}

}  // namespace backend
}  // namespace delphyne
