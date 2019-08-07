
// Copyright 2018 Toyota Research Institute

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
  test::IgnMonitor<ignition::msgs::Model_V> ign_monitor(kTopicName);

  // Creates a simulator to work with the publisher.
  drake::systems::Simulator<double> simulator(ign_publisher, ign_publisher.CreateDefaultContext());

  // Configures context's input with the pre-loaded message.
  simulator.get_mutable_context().FixInputPort(0, drake::AbstractValue::Make(kIgnMsg));

  // Simulates for a small time period.
  simulator.Initialize();
  const double kPublishDeadline{0.1};
  simulator.StepTo(kPublishDeadline);

  // Checks that the correct amount of messages have been published.
  const int kMessagesToPublish = simulator.get_num_steps_taken();
  ASSERT_TRUE(ign_monitor.wait_until(kMessagesToPublish, kTimeoutMs));

  // Verifies the equivalence of the original ignition message
  // and the received one.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(kIgnMsg, ign_monitor.get_last_message()));
}

// Creates an Ignition Publisher System and publish a message repeatedly at a
// low frequency, then checks that it has been received the correct amount of
// times.
TEST_F(IgnPublisherSystemTest, LowFrequencyPublishTest) {
  const double kPublishRateHz{4.0};
  // Sets up publisher system and monitor subscription.
  IgnPublisherSystem<ignition::msgs::Model_V> ign_publisher(kTopicName, kPublishRateHz);
  test::IgnMonitor<ignition::msgs::Model_V> ign_monitor(kTopicName);

  // Creates a simulator to work with the publisher.
  drake::systems::Simulator<double> simulator(ign_publisher, ign_publisher.CreateDefaultContext());

  // Configures context's input with the pre-loaded message.
  simulator.get_mutable_context().FixInputPort(0, drake::AbstractValue::Make(kIgnMsg));

  // Simulates enough for the expected message count to get
  // published.
  simulator.Initialize();
  const int kMessagesToPublish{5};
  const double kPublishDeadline = kMessagesToPublish / kPublishRateHz;
  simulator.StepTo(kPublishDeadline);

  // Checks that the correct amount of messages have been published.
  ASSERT_TRUE(ign_monitor.wait_until(kMessagesToPublish, kTimeoutMs));

  // Verifies the equivalence of the original ignition message
  // and the received one.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(kIgnMsg, ign_monitor.get_last_message()));
}

}  // namespace
}  // namespace delphyne
