// Copyright 2017 Toyota Research Institute

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
