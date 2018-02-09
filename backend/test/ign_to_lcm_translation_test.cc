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

}  // namespace backend
}  // namespace delphyne
