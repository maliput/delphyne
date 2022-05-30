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

#include "translations/lcm_viewer_draw_to_ign_model_v.h"

#include <map>

#include <maliput/common/maliput_unused.h>

#include "translations/generate_unique_id.h"
#include "translations/time_conversion.h"

namespace delphyne {

void LcmViewerDrawToIgnModelV::DoDrakeToIgnTranslation(const drake::lcmt_viewer_draw& lcm_message,
                                                       ignition::msgs::Model_V* ign_message, int64_t time) const {
  maliput::common::unused(time);

  DELPHYNE_VALIDATE(lcm_message.link_name.size() == static_cast<unsigned int>(lcm_message.num_links),
                    std::invalid_argument, "LCM link name size must equal the number of links");
  DELPHYNE_VALIDATE(lcm_message.robot_num.size() == static_cast<unsigned int>(lcm_message.num_links),
                    std::invalid_argument, "LCM robot number size must equal the number of links");
  DELPHYNE_VALIDATE(lcm_message.position.size() == static_cast<unsigned int>(lcm_message.num_links),
                    std::invalid_argument, "LCM position size must equal the number of links");
  DELPHYNE_VALIDATE(lcm_message.quaternion.size() == static_cast<unsigned int>(lcm_message.num_links),
                    std::invalid_argument, "LCM quaternion size must equal the number of links");

  // Clears state from the previous call.
  // @see DrakeToIgn::DoDrakeToIgnTranslation
  ign_message->Clear();

  // LCM timestamps are in milliseconds.
  ign_message->mutable_header()->mutable_stamp()->CopyFrom(MillisToIgnitionTime(lcm_message.timestamp));

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
    DELPHYNE_VALIDATE(lcm_message.position[i].size() == kPositionVectorSize, std::runtime_error,
                      "Position vector size did not match");
    PositionArrayToIgnition(lcm_message.position[i].data(), pose->mutable_position());

    // Checks orientation size and translates.
    DELPHYNE_VALIDATE(lcm_message.quaternion[i].size() == kOrientationVectorSize, std::runtime_error,
                      "Orientation vector size did not match");
    QuaternionArrayToIgnition(lcm_message.quaternion[i].data(), pose->mutable_orientation());

    // Add unique integer id per link
    const size_t linkId = GenerateLinkId(robotId, link->name());

    link->set_id(linkId);
    pose->set_id(linkId);
  }
}

}  // namespace delphyne
