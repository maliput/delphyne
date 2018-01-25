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

#include <drake/lcmt_viewer_draw.hpp>

#include <ignition/msgs.hh>

#include "gtest/gtest.h"

namespace delphyne {
namespace backend {

// Generates a pre-loaded lcmt_viewer_draw message.
drake::lcmt_viewer_draw get_preloaded_draw_msg() {
  drake::lcmt_viewer_draw msg;
  msg.timestamp = 0;
  msg.num_links = 1;
  msg.link_name.resize(msg.num_links);
  msg.link_name[0] = "box";
  msg.robot_num.resize(1);
  msg.robot_num[0] = 1;
  msg.position.resize(1);
  msg.position[0].resize(3);
  msg.position[0][0] = 0.0;
  msg.position[0][1] = 0.0;
  msg.position[0][2] = 0.0;
  msg.quaternion.resize(1);
  msg.quaternion[0].resize(4);
  msg.quaternion[0][0] = 0.0;
  msg.quaternion[0][1] = 0.0;
  msg.quaternion[0][2] = 0.0;
  msg.quaternion[0][3] = 1.0;
  return msg;
}

// Checks that all the array-iterable values from
// lcmt_viewer_draw matches their ignition counterpart.
void CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                         const ignition::msgs::Model_V& ign_models) {
  for (int i = 0; i < lcm_msg.num_links; i++) {
    // Step 1: Checks there is a corresponding ignition model for the LCM link.
    ignition::msgs::Model model;
    for (int j = 0; j < ign_models.models_size(); ++j) {
      if (ign_models.models(j).id() == (unsigned)lcm_msg.robot_num[i]) {
        model = ign_models.models(j);
      }
    }
    ASSERT_NE(nullptr, &model);

    // Step 2: Checks there is a corresponding ignition link for the LCM link.
    ignition::msgs::Link link;
    for (int j = 0; j < model.link_size(); ++j) {
      if (model.link(j).name() == lcm_msg.link_name[i]) {
        link = model.link(j);
      }
    }
    ASSERT_NE(nullptr, &link);

    // Step 3: Gets the pose and compares the values.
    ignition::msgs::Pose pose = link.pose();

    EXPECT_EQ(pose.position().x(), lcm_msg.position[i][0]);
    EXPECT_EQ(pose.position().y(), lcm_msg.position[i][1]);
    EXPECT_EQ(pose.position().z(), lcm_msg.position[i][2]);
    EXPECT_EQ(pose.orientation().w(), lcm_msg.quaternion[i][0]);
    EXPECT_EQ(pose.orientation().x(), lcm_msg.quaternion[i][1]);
    EXPECT_EQ(pose.orientation().y(), lcm_msg.quaternion[i][2]);
    EXPECT_EQ(pose.orientation().z(), lcm_msg.quaternion[i][3]);
  }
}

}  // namespace backend
}  // namespace delphyne
