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

#include "backend/ign_publisher_system.h"

#include <chrono>

#include <drake/common/value.h>
#include <drake/systems/analysis/simulator.h>
#include <gtest/gtest.h>
#include <ignition/msgs.hh>

#include "test_utilities/helpers.h"

namespace delphyne {
namespace {

class IgnPublisherSystemTest : public ::testing::Test {
 protected:
  // Ignition transport topic name to subscribe/publish to.
  const std::string kTopicName{"visualizer/scene_update"};
  // Dummy ignition message for testing purposes.
  const ignition::msgs::Model_V kIgnMsg{test::BuildPreloadedModelVMsg()};
  const std::chrono::milliseconds kTimeoutMs{100};
};

// Creates an Ignition Publisher System and publish a message, then checks that
// it has been correctly received.
TEST_F(IgnPublisherSystemTest, ImmediatePublishTest) {
  // Sets up publisher system and monitor subscription.
  IgnPublisherSystem<ignition::msgs::Model_V> ign_publisher(kTopicName);
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Model_V>(kTopicName);

  // Creates a simulator to work with the publisher.
  drake::systems::Simulator<double> simulator(ign_publisher, ign_publisher.CreateDefaultContext());

  // Configures context's input with the pre-loaded message.
  simulator.get_mutable_context().FixInputPort(0, drake::Value<ignition::msgs::Model_V>(kIgnMsg));

  // Simulates for a small time period.
  simulator.Initialize();
  const double kPublishDeadline{0.1};
  simulator.AdvanceTo(kPublishDeadline);

  // Checks that the correct amount of messages have been published.
  const int kMessagesToPublish = simulator.get_num_steps_taken();
  ASSERT_TRUE(ign_monitor->wait_until(kMessagesToPublish, kTimeoutMs));

  // Verifies the equivalence of the original ignition message
  // and the received one.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(kIgnMsg, ign_monitor->get_last_message()));
}

// Creates an Ignition Publisher System and publish a message repeatedly at a
// low frequency, then checks that it has been received the correct amount of
// times.
TEST_F(IgnPublisherSystemTest, LowFrequencyPublishTest) {
  const double kPublishRateHz{4.0};
  // Sets up publisher system and monitor subscription.
  IgnPublisherSystem<ignition::msgs::Model_V> ign_publisher(kTopicName, kPublishRateHz);
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Model_V>(kTopicName);

  // Creates a simulator to work with the publisher.
  drake::systems::Simulator<double> simulator(ign_publisher, ign_publisher.CreateDefaultContext());

  // Configures context's input with the pre-loaded message.
  simulator.get_mutable_context().FixInputPort(0, drake::Value<ignition::msgs::Model_V>(kIgnMsg));

  // Simulates enough for the expected message count to get
  // published.
  simulator.Initialize();
  const int kMessagesToPublish{5};
  const double kPublishDeadline = kMessagesToPublish / kPublishRateHz;
  simulator.AdvanceTo(kPublishDeadline);

  // Checks that the correct amount of messages have been published.
  ASSERT_TRUE(ign_monitor->wait_until(kMessagesToPublish, kTimeoutMs));

  // Verifies the equivalence of the original ignition message
  // and the received one.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(kIgnMsg, ign_monitor->get_last_message()));
}

}  // namespace
}  // namespace delphyne
