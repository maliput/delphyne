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

#include "bridge/lcm_channel_repeater.h"

namespace delphyne {
namespace bridge {

static bool handler1Called;
static bool handler2Called;

//////////////////////////////////////////////////
/// \brief Assert that a given ign Geometry is a box and has
/// the specified size
void assertIsBoxWithSize(const ignition::msgs::Geometry& message, float x,
                         float y, float z) {
  ASSERT_EQ(message.type(), ignition::msgs::Geometry::BOX);
  ASSERT_EQ(message.box().size().x(), x);
  ASSERT_EQ(message.box().size().y(), y);
  ASSERT_EQ(message.box().size().z(), z);
}

//////////////////////////////////////////////////
/// \brief Fill an LCM viewer_geometry_data message with a box
/// geometry with the specified size
void fillBoxWith(drake::lcmt_viewer_geometry_data* boxMsg, float x, float y,
                 float z) {
  boxMsg->type = boxMsg->BOX;
  boxMsg->num_float_data = 3;
  boxMsg->float_data.resize(boxMsg->num_float_data);
  boxMsg->float_data[0] = x;
  boxMsg->float_data[1] = y;
  boxMsg->float_data[2] = z;
}

//////////////////////////////////////////////////
/// \brief Handler called in TEST_CHANNEL_1. Checks that the
/// box message has the expected size and flags the handler as
/// called
void test1Handler(const ignition::msgs::Geometry& message) {
  assertIsBoxWithSize(message, 1, 2, 3);
  handler1Called = true;
}

//////////////////////////////////////////////////
/// \brief Handler called in TEST_CHANNEL_2. Checks that the
/// box message has the expected size and flags the handler as
/// called
void test2Handler(const ignition::msgs::Geometry& message) {
  assertIsBoxWithSize(message, 5, 5, 5);
  handler2Called = true;
}

//////////////////////////////////////////////////
/// \brief To be called before a test, clears the handlers
/// called flags
void reset() {
  handler1Called = false;
  handler2Called = false;
}

//////////////////////////////////////////////////
/// \brief Test how a message is processed end-to-end by using
/// a repeater.
GTEST_TEST(LCMChannelRepeaterTest, TestEndToEndEcho) {
  reset();

  ASSERT_FALSE(handler1Called);

  // Setup LCM
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Setup repeater and hook handler to ignition topic
  delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_geometry_data,
                                       ignition::msgs::Geometry>
      testRepeater(lcm, "TEST_CHANNEL_1");

  testRepeater.Start();

  ignition::transport::Node node;
  node.Subscribe("TEST_CHANNEL_1", test1Handler);

  // Create and publish geometry message
  drake::lcmt_viewer_geometry_data boxMsg;
  fillBoxWith(&boxMsg, 1, 2, 3);

  lcm->publish("TEST_CHANNEL_1", &boxMsg);

  // Consume LCM message
  lcm->handleTimeout(500);

  // Check handler has been called
  ASSERT_TRUE(handler1Called);
}

//////////////////////////////////////////////////
/// \brief Test that if the Start() method is not called on the
/// repeater no handler will be setup to listen to the LCM channel.
GTEST_TEST(LCMChannelRepeaterTest, TestHandlerNotCalled) {
  reset();

  ASSERT_FALSE(handler1Called);

  // Setup LCM
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Setup repeater and hook handler to ignition topic
  delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_geometry_data,
                                       ignition::msgs::Geometry>
      testRepeater(lcm, "TEST_CHANNEL_1");

  ignition::transport::Node node;
  node.Subscribe("TEST_CHANNEL_1", test1Handler);

  // Create and publish geometry message
  drake::lcmt_viewer_geometry_data boxMsg;
  fillBoxWith(&boxMsg, 1, 2, 3);

  lcm->publish("TEST_CHANNEL_1", &boxMsg);

  // Consume LCM message
  lcm->handleTimeout(500);

  // Check handler has not been called
  ASSERT_FALSE(handler1Called);
}

//////////////////////////////////////////////////
/// \brief Test that two repeaters can be safely setup on different
/// channels using the same LCM instance to dispatch all the messages.
GTEST_TEST(LCMChannelRepeaterTest, TestMultipleChannels) {
  reset();

  ASSERT_FALSE(handler1Called);
  ASSERT_FALSE(handler2Called);

  // Setup LCM
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Setup repeater 1 and hook handler to ignition topic
  delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_geometry_data,
                                       ignition::msgs::Geometry>
      channel1Repeater(lcm, "TEST_CHANNEL_1");

  channel1Repeater.Start();

  ignition::transport::Node node1;
  node1.Subscribe("TEST_CHANNEL_1", test1Handler);

  // Setup repeater 2 and hook handler to ignition topic
  delphyne::bridge::LcmChannelRepeater<drake::lcmt_viewer_geometry_data,
                                       ignition::msgs::Geometry>
      channel2Repeater(lcm, "TEST_CHANNEL_2");

  channel2Repeater.Start();

  ignition::transport::Node node2;
  node2.Subscribe("TEST_CHANNEL_2", test2Handler);

  // Create and publish geometry message to channel 1
  drake::lcmt_viewer_geometry_data box1Msg;
  fillBoxWith(&box1Msg, 1, 2, 3);

  lcm->publish("TEST_CHANNEL_1", &box1Msg);

  // Create and publish geometry message to channel 2
  drake::lcmt_viewer_geometry_data box2Msg;
  fillBoxWith(&box2Msg, 5, 5, 5);

  lcm->publish("TEST_CHANNEL_2", &box2Msg);

  // Consume LCM messages
  lcm->handleTimeout(500);
  lcm->handleTimeout(500);

  // Check handlers have been called
  ASSERT_TRUE(handler1Called);
  ASSERT_TRUE(handler2Called);
}

}  // namespace bridge
}  // namespace delphyne
