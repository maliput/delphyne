#include <iostream>
#include <gtest/gtest.h>
#include <ignition/msgs.hh>

#include "drake/lcmt_driving_command_t.hpp"
#include "bridge/ign_to_lcm_translation.hh"
#include "protobuf/automotive_driving_command.pb.h"

namespace delphyne {
namespace bridge {

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

}  // namespace bridge
}  // namespace delphyne
