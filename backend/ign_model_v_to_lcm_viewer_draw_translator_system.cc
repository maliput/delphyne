// Copyright 2018 Toyota Research Institute

#include "backend/ign_model_v_to_lcm_viewer_draw_translator_system.h"

namespace delphyne {
namespace backend {

IgnModelVToLcmViewerDrawTranslatorSystem::
    IgnModelVToLcmViewerDrawTranslatorSystem() {
  InitPorts();
}

void IgnModelVToLcmViewerDrawTranslatorSystem::DoIgnToLcmTranslation(
    const ignition::msgs::Model_V& ign_message,
    drake::lcmt_viewer_draw* lcm_message) const {
  // Clear old state
  lcm_message->num_links = 0;
  lcm_message->link_name.clear();
  lcm_message->robot_num.clear();
  lcm_message->position.clear();
  lcm_message->quaternion.clear();

  lcm_message->timestamp = ignTimeToTimestamp(ign_message.header().stamp());

  // Each ignition model has many links using the same id, but this is
  // flattened in LCM
  for (int i = 0; i < ign_message.models_size(); ++i) {
    const ::ignition::msgs::Model& robot_model = ign_message.models(i);

    for (int j = 0; j < robot_model.link_size(); ++j) {
      lcm_message->robot_num.push_back(robot_model.id());

      lcm_message->num_links += 1;

      const ::ignition::msgs::Link& link = robot_model.link(j);
      lcm_message->link_name.push_back(link.name());

      const ::ignition::msgs::Pose& pose = link.pose();
      lcm_message->position.push_back(ignPositionToVector(pose.position()));
      lcm_message->quaternion.push_back(
          ignQuaternionToVector(pose.orientation()));
    }
  }
}

}  // namespace backend
}  // namespace delphyne
