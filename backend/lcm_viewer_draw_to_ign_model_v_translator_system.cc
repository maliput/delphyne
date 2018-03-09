// Copyright 2018 Open Source Robotics Foundation
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

#include <map>
#include <string>

#include "backend/lcm_viewer_draw_to_ign_model_v_translator_system.h"

namespace delphyne {
namespace backend {

LcmViewerDrawToIgnModelVTranslatorSystem::
    LcmViewerDrawToIgnModelVTranslatorSystem() {
  InitPorts();
}

void LcmViewerDrawToIgnModelVTranslatorSystem::DoLcmToIgnTranslation(
    const drake::lcmt_viewer_draw& lcm_message,
    ignition::msgs::Model_V* ign_message) const {
  DELPHYNE_DEMAND(lcm_message.link_name.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));
  DELPHYNE_DEMAND(lcm_message.robot_num.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));
  DELPHYNE_DEMAND(lcm_message.position.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));
  DELPHYNE_DEMAND(lcm_message.quaternion.size() ==
                  static_cast<unsigned int>(lcm_message.num_links));

  LcmTimestampToIgnition(lcm_message.timestamp,
                         ign_message->mutable_header()->mutable_stamp());

  std::map<int32_t, ignition::msgs::Model*> models;

  // Add one pose per link
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

    // Check position size and translate
    DELPHYNE_DEMAND(lcm_message.position[i].size() == kPositionVectorSize);
    LcmPositionToIgnition(lcm_message.position[i].data(),
                          pose->mutable_position());

    // Check orientation size and translate
    DELPHYNE_DEMAND(lcm_message.quaternion[i].size() == kOrientationVectorSize);
    LcmQuaternionToIgnition(lcm_message.quaternion[i].data(),
                            pose->mutable_orientation());
  }
}

}  // namespace backend
}  // namespace delphyne
