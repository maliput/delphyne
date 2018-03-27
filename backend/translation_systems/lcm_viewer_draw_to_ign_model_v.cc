// Copyright 2018 Toyota Research Institute

#include "backend/translation_systems/lcm_viewer_draw_to_ign_model_v.h"

#include <map>
#include <string>

#include "backend/time_conversion.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

void LcmViewerDrawToIgnModelV::DoDrakeToIgnTranslation(
    const drake::lcmt_viewer_draw& lcm_message,
    ignition::msgs::Model_V* ign_message, int64_t time) const {
  DELPHYNE_DEMAND(lcm_message.link_name.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));
  DELPHYNE_DEMAND(lcm_message.robot_num.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));
  DELPHYNE_DEMAND(lcm_message.position.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));
  DELPHYNE_DEMAND(lcm_message.quaternion.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));

  // LCM timestamps are in milliseconds.
  ign_message->mutable_header()->mutable_stamp()->CopyFrom(
      MillisToIgnitionTime(lcm_message.timestamp));

  // Clears state from the previous call.
  // @see LcmToIgn::DoDrakeToIgnTranslation
  ign_message->Clear();

  std::map<int32_t, ignition::msgs::Model*> models;

  // Adds one pose per link.
  for (int i = 0; i < lcm_message.num_links; ++i) {
    const int32_t robotId = lcm_message.robot_num[i];
    if (models.count(robotId) == 0) {
      models[robotId] = ign_message->add_models();
      models[robotId]->set_id(robotId);
    }

    ignition::msgs::Model* robotModel = models[robotId];

    ignition::msgs::Link* link = robotModel->add_link();
    link->set_name(lcm_message.link_name[i]);

    ignition::msgs::Pose* pose = link->mutable_pose();

    // Checks position size and translates.
    DELPHYNE_DEMAND(lcm_message.position[i].size() == kPositionVectorSize);
    LcmPositionToIgnition(lcm_message.position[i].data(),
                          pose->mutable_position());

    // Checks orientation size and translates.
    DELPHYNE_DEMAND(lcm_message.quaternion[i].size() == kOrientationVectorSize);
    LcmQuaternionToIgnition(lcm_message.quaternion[i].data(),
                            pose->mutable_orientation());
  }
}

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
