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
