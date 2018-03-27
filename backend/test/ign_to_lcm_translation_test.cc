// Copyright 2017 Toyota Research Institute

#include "backend/ign_to_lcm_translation.h"

#include <iostream>

#include "backend/test/helpers.h"
#include "drake/lcmt_driving_command_t.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "gtest/gtest.h"
#include "ignition/msgs.hh"
#include "protobuf/automotive_driving_command.pb.h"

namespace delphyne {
namespace backend {

/////////////////////////////////////////////////
// \brief Test that the ignition AutomotiveDrivingCommand
// message is properly translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestAutomotiveDrivingCommandTranslation) {
  // Defines the ignition command
  ignition::msgs::AutomotiveDrivingCommand ign_driving_message;
  // Defines LCM expected message
  drake::lcmt_driving_command_t lcm_driving_message;
  // Fills LCM data
  ign_driving_message.mutable_time()->set_sec(123);
  ign_driving_message.mutable_time()->set_nsec(987222111);
  ign_driving_message.set_theta(0.12);
  ign_driving_message.set_acceleration(15.7);

  // Translates from ignition to LCM
  ignToLcm(ign_driving_message, &lcm_driving_message);

  // Verifies generated LCM message
  EXPECT_EQ(123987, lcm_driving_message.timestamp);
  EXPECT_EQ(0.12, lcm_driving_message.steering_angle);
  EXPECT_EQ(15.7, lcm_driving_message.acceleration);
}

/////////////////////////////////////////////////
// \brief Test that the ignition AutomotiveDrivingCommand
// message is properly translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestAutomotiveDrivingCommandTranslationDefaultValues) {
  // Defines the ignition command
  ignition::msgs::AutomotiveDrivingCommand ign_driving_message;
  // Defines LCM expected message
  drake::lcmt_driving_command_t lcm_driving_message;

  // Translates from ignition to LCM
  ignToLcm(ign_driving_message, &lcm_driving_message);

  // Verifies generated LCM message
  EXPECT_EQ(lcm_driving_message.steering_angle, 0);
  EXPECT_EQ(lcm_driving_message.acceleration, 0);
}

/////////////////////////////////////////////////
// \brief Test that the ignition Robot Model message is properly
// translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestRobotModelTranslation) {
  ignition::msgs::Model_V robot_models;

  robot_models.mutable_header()->mutable_stamp()->set_sec(123);
  robot_models.mutable_header()->mutable_stamp()->set_nsec(456000000);

  for (int i = 0; i < 3; ++i) {
    ::ignition::msgs::Model* model = robot_models.add_models();
    model->set_id(i);

    for (int j = 0; j < 2; ++j) {
      ::ignition::msgs::Link* link = model->add_link();

      ::ignition::msgs::Pose* pose = link->mutable_pose();
      ignition::msgs::Vector3d* position = pose->mutable_position();
      ignition::msgs::Quaternion* orientation = pose->mutable_orientation();

      link->set_name(std::to_string(i) + std::to_string(j));

      position->set_x(i);
      position->set_y(j + 5.0);
      position->set_z(i + 10.0);

      orientation->set_w(i);
      orientation->set_x(j + 5.0);
      orientation->set_y(i + 10.0);
      orientation->set_z(j + 15.0);
    }
  }

  drake::lcmt_viewer_draw robot_draw_data;
  ignToLcm(robot_models, &robot_draw_data);

  EXPECT_EQ(robot_draw_data.timestamp, 123456);

  EXPECT_EQ(robot_draw_data.num_links, 6);
  EXPECT_EQ(robot_draw_data.robot_num.size(), 6);
  EXPECT_EQ(robot_draw_data.link_name.size(), 6);
  EXPECT_EQ(robot_draw_data.position.size(), 6);
  EXPECT_EQ(robot_draw_data.quaternion.size(), 6);

  EXPECT_TRUE(
      delphyne::test::CheckMsgTranslation(robot_draw_data, robot_models));
}

/////////////////////////////////////////////////
// \brief Test that the ignition Robot Model message is properly
// translated to its LCM counterpart
GTEST_TEST(ignToLcm, TestRobotModelTranslationDefaultValues) {
  ignition::msgs::Model_V robot_models;
  drake::lcmt_viewer_draw robot_draw_data;

  ignToLcm(robot_models, &robot_draw_data);

  EXPECT_EQ(robot_draw_data.num_links, 0);
  EXPECT_EQ(robot_draw_data.robot_num.size(), 0);
  EXPECT_EQ(robot_draw_data.link_name.size(), 0);
  EXPECT_EQ(robot_draw_data.position.size(), 0);
  EXPECT_EQ(robot_draw_data.quaternion.size(), 0);
}

}  // namespace backend
}  // namespace delphyne