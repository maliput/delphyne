// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "translations/ign_model_v_to_lcm_viewer_draw.h"

#include "translations/time_conversion.h"

namespace delphyne {

void IgnModelVToLcmViewerDraw::DoIgnToDrakeTranslation(const ignition::msgs::Model_V& ign_message,
                                                       drake::lcmt_viewer_draw* lcm_message) const {
  // Clears state from the previous call.
  // @see IgnToDrake::DoIgnToDrakeTranslation
  lcm_message->link_name.clear();
  lcm_message->robot_num.clear();
  lcm_message->position.clear();
  lcm_message->quaternion.clear();
  lcm_message->num_links = 0;

  // LCM timestamps are in milliseconds.
  const ignition::msgs::Time& ign_timestamp = ign_message.header().stamp();
  lcm_message->timestamp = SecsAndNanosToMillis(ign_timestamp.sec(), ign_timestamp.nsec());

  // Ignition models have a hierarchical structure, where each model contains
  // its many links, whereas LCM flattens this into a single array, using the id
  // field to tell different models apart.
  for (int i = 0; i < ign_message.models_size(); ++i) {
    const ::ignition::msgs::Model& robot_model = ign_message.models(i);

    for (int j = 0; j < robot_model.link_size(); ++j) {
      lcm_message->robot_num.push_back(robot_model.id());

      const ::ignition::msgs::Link& link = robot_model.link(j);
      lcm_message->link_name.push_back(link.name());

      const ::ignition::msgs::Pose& pose = link.pose();
      lcm_message->position.push_back(IgnPositionToVector(pose.position()));
      lcm_message->quaternion.push_back(IgnQuaternionToVector(pose.orientation()));

      lcm_message->num_links += 1;
    }
  }
}

}  // namespace delphyne
