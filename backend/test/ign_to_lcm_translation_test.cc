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

#include <iostream>
#include <gtest/gtest.h>
#include <ignition/msgs.hh>

#include "backend/ign_to_lcm_translation.h"
#include "drake/lcmt_driving_command_t.hpp"
#include <drake/lcmt_viewer_draw.hpp>
#include <ignition/msgs.hh>
#include "protobuf/automotive_driving_command.pb.h"

namespace delphyne {
namespace backend {

//////////////////////////////////////////////////
/// \brief Test that the ignition AutomotiveDrivingCommand
/// message is properly translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestAutomotiveDrivingCommandTranslation) {
  // Define the ignition command
  ignition::msgs::AutomotiveDrivingCommand ignDrivingMsg;
  // Define LCM expected message
  drake::lcmt_driving_command_t lcmDrivingMsg;
  // Fill LCM data
  ignDrivingMsg.mutable_time()->set_sec(123);
  ignDrivingMsg.mutable_time()->set_nsec(987222111);
  ignDrivingMsg.set_theta(0.12);
  ignDrivingMsg.set_acceleration(15.7);

  // Translate from ignition to LCM
  ignToLcm(ignDrivingMsg, &lcmDrivingMsg);

  // Verify generated LCM message
  EXPECT_EQ(123987, lcmDrivingMsg.timestamp);
  EXPECT_EQ(0.12, lcmDrivingMsg.steering_angle);
  EXPECT_EQ(15.7, lcmDrivingMsg.acceleration);
}

//////////////////////////////////////////////////
/// \brief Test that the ignition AutomotiveDrivingCommand
/// message is properly translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestAutomotiveDrivingCommandTranslationDefaultValues) {
  // Define the ignition command
  ignition::msgs::AutomotiveDrivingCommand ignDrivingMsg;
  // Define LCM expected message
  drake::lcmt_driving_command_t lcmDrivingMsg;

  // Translate from ignition to LCM
  ignToLcm(ignDrivingMsg, &lcmDrivingMsg);

  // Verify generated LCM message
  EXPECT_EQ(lcmDrivingMsg.steering_angle, 0);
  EXPECT_EQ(lcmDrivingMsg.acceleration, 0);
}

//////////////////////////////////////////////////
/// \brief Test that the ignition Robot Model message is properly
/// translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestRobotModelTranslation) {
  ignition::msgs::Model_V robotModels;

  robotModels.mutable_header()->mutable_stamp()->set_sec(123);
  robotModels.mutable_header()->mutable_stamp()->set_nsec(456000000);

  for (int i = 0; i < 3; ++i) {
    auto model = robotModels.add_models();
    model->set_id(i);

    for (int j = 0; j < 2; ++j) {
      auto link = model->add_link();

      auto pose = link->mutable_pose();
      auto position = pose->mutable_position();
      auto orientation = pose->mutable_orientation();

      link->set_name(std::to_string(i) + std::to_string(j));

      position->set_x(i);
      position->set_y(j + 5);
      position->set_z(i + 10);

      orientation->set_w(i);
      orientation->set_x(j + 5);
      orientation->set_y(i + 10);
      orientation->set_z(j + 15);
    }
  }

  drake::lcmt_viewer_draw robotDrawData;
  ignToLcm(robotModels, &robotDrawData);

  EXPECT_EQ(robotDrawData.timestamp, 123456);

  EXPECT_EQ(robotDrawData.num_links, 6);
  EXPECT_EQ(robotDrawData.robot_num.size(), 6);
  EXPECT_EQ(robotDrawData.link_name.size(), 6);
  EXPECT_EQ(robotDrawData.position.size(), 6);
  EXPECT_EQ(robotDrawData.quaternion.size(), 6);

  // Model 0 link 0
  EXPECT_EQ(robotDrawData.robot_num[0], 0);
  EXPECT_EQ(robotDrawData.link_name[0], "00");

  EXPECT_EQ(robotDrawData.position[0][0], 0);
  EXPECT_EQ(robotDrawData.position[0][1], 5);
  EXPECT_EQ(robotDrawData.position[0][2], 10);

  EXPECT_EQ(robotDrawData.quaternion[0][0], 0);
  EXPECT_EQ(robotDrawData.quaternion[0][1], 5);
  EXPECT_EQ(robotDrawData.quaternion[0][2], 10);
  EXPECT_EQ(robotDrawData.quaternion[0][3], 15);

  // Model 0 link 1
  EXPECT_EQ(robotDrawData.robot_num[1], 0);
  EXPECT_EQ(robotDrawData.link_name[1], "01");

  EXPECT_EQ(robotDrawData.position[1][0], 0);
  EXPECT_EQ(robotDrawData.position[1][1], 6);
  EXPECT_EQ(robotDrawData.position[1][2], 10);

  EXPECT_EQ(robotDrawData.quaternion[1][0], 0);
  EXPECT_EQ(robotDrawData.quaternion[1][1], 6);
  EXPECT_EQ(robotDrawData.quaternion[1][2], 10);
  EXPECT_EQ(robotDrawData.quaternion[1][3], 16);

  // Model 1 link 0
  EXPECT_EQ(robotDrawData.robot_num[2], 1);
  EXPECT_EQ(robotDrawData.link_name[2], "10");

  EXPECT_EQ(robotDrawData.position[2][0], 1);
  EXPECT_EQ(robotDrawData.position[2][1], 5);
  EXPECT_EQ(robotDrawData.position[2][2], 11);

  EXPECT_EQ(robotDrawData.quaternion[2][0], 1);
  EXPECT_EQ(robotDrawData.quaternion[2][1], 5);
  EXPECT_EQ(robotDrawData.quaternion[2][2], 11);
  EXPECT_EQ(robotDrawData.quaternion[2][3], 15);

  // Model 1 link 1
  EXPECT_EQ(robotDrawData.robot_num[3], 1);
  EXPECT_EQ(robotDrawData.link_name[3], "11");

  EXPECT_EQ(robotDrawData.position[3][0], 1);
  EXPECT_EQ(robotDrawData.position[3][1], 6);
  EXPECT_EQ(robotDrawData.position[3][2], 11);

  EXPECT_EQ(robotDrawData.quaternion[3][0], 1);
  EXPECT_EQ(robotDrawData.quaternion[3][1], 6);
  EXPECT_EQ(robotDrawData.quaternion[3][2], 11);
  EXPECT_EQ(robotDrawData.quaternion[3][3], 16);

  // Model 2 link 0
  EXPECT_EQ(robotDrawData.robot_num[4], 2);
  EXPECT_EQ(robotDrawData.link_name[4], "20");

  EXPECT_EQ(robotDrawData.position[4][0], 2);
  EXPECT_EQ(robotDrawData.position[4][1], 5);
  EXPECT_EQ(robotDrawData.position[4][2], 12);

  EXPECT_EQ(robotDrawData.quaternion[4][0], 2);
  EXPECT_EQ(robotDrawData.quaternion[4][1], 5);
  EXPECT_EQ(robotDrawData.quaternion[4][2], 12);
  EXPECT_EQ(robotDrawData.quaternion[4][3], 15);

  // Model 2 link 1
  EXPECT_EQ(robotDrawData.robot_num[5], 2);
  EXPECT_EQ(robotDrawData.link_name[5], "21");

  EXPECT_EQ(robotDrawData.position[5][0], 2);
  EXPECT_EQ(robotDrawData.position[5][1], 6);
  EXPECT_EQ(robotDrawData.position[5][2], 12);

  EXPECT_EQ(robotDrawData.quaternion[5][0], 2);
  EXPECT_EQ(robotDrawData.quaternion[5][1], 6);
  EXPECT_EQ(robotDrawData.quaternion[5][2], 12);
  EXPECT_EQ(robotDrawData.quaternion[5][3], 16);
}

//////////////////////////////////////////////////
/// \brief Test that the ignition Robot Model message is properly
/// translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestRobotModelTranslationDefaultValues) {
  ignition::msgs::Model_V robotModels;
  drake::lcmt_viewer_draw robotDrawData;

  ignToLcm(robotModels, &robotDrawData);

  EXPECT_EQ(robotDrawData.num_links, 0);
  EXPECT_EQ(robotDrawData.robot_num.size(), 0);
  EXPECT_EQ(robotDrawData.link_name.size(), 0);
  EXPECT_EQ(robotDrawData.position.size(), 0);
  EXPECT_EQ(robotDrawData.quaternion.size(), 0);
}

}  // namespace backend
}  // namespace delphyne
