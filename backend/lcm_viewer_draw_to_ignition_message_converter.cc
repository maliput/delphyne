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

#include "backend/lcm_viewer_draw_to_ignition_message_converter.h"

namespace delphyne {
namespace backend {

void LCMViewerDrawToIgnitionMessageConverter::LcmToIgn(
    const drake::lcmt_viewer_draw& robotDrawData,
    ignition::msgs::Model_V* robotModels) {
  // Check the size of each vector on an lcm_viewer_draw message
  // num_links represents the amount of links declared and
  // should be matched by the size of each of the following vectors
  checkVectorSize(robotDrawData.link_name.size(), robotDrawData.num_links,
                  "link_name");
  checkVectorSize(robotDrawData.robot_num.size(), robotDrawData.num_links,
                  "robot_num");
  checkVectorSize(robotDrawData.position.size(), robotDrawData.num_links,
                  "position");
  checkVectorSize(robotDrawData.quaternion.size(), robotDrawData.num_links,
                  "quaternion");

  lcmToIgn(robotDrawData.timestamp,
           robotModels->mutable_header()->mutable_stamp());

  std::map<int32_t, ignition::msgs::Model*> models;

  // Add one pose per link
  for (int i = 0; i < robotDrawData.num_links; ++i) {
    int32_t robotId = robotDrawData.robot_num[i];
    if (models.count(robotId) == 0) {
      models[robotId] = robotModels->add_models();
      models[robotId]->set_id(robotId);
    }

    ignition::msgs::Model* robotModel = models[robotId];
    ignition::msgs::Link* link = robotModel->add_link();
    ignition::msgs::Pose* pose = link->mutable_pose();

    link->set_name(robotDrawData.link_name[i]);

    // Check position size and translate
    checkVectorSize(robotDrawData.position[i].size(), 3,
                    "position[" + std::to_string(i) + "]");
    lcmToIgn(robotDrawData.position[i].data(), pose->mutable_position());

    // Check orientation size and translate
    checkVectorSize(robotDrawData.quaternion[i].size(), 4,
                    "quaternion[" + std::to_string(i) + "]");
    lcmToIgn(robotDrawData.quaternion[i].data(), pose->mutable_orientation());
  }
};

void LCMViewerDrawToIgnitionMessageConverter::IgnToLcm(
    const ignition::msgs::Model_V& robotModels,
    drake::lcmt_viewer_draw* robotDrawData) {
  ignToLcm(robotModels.header().stamp(), &(robotDrawData->timestamp));

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
};

void LCMViewerDrawToIgnitionMessageConverter::checkVectorSize(
    int vectorSize, int expectedSize, std::string fieldName) {
  if (vectorSize != expectedSize) {
    std::stringstream message;
    message << "Wrong size for " << fieldName << ": expecting " << expectedSize
            << " elements but " << vectorSize << " given.";
    throw TranslateException(message.str());
  }
}

}  // namespace backend
}  // namespace delphyne
