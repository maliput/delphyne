// Copyright 2018 Toyota Research Institute

#include "backend/ign_publisher_system.h"

#include <chrono>
#include <thread>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/value.h>

#include <gtest/gtest.h>

#include <ignition/msgs.hh>

#include "helpers.h"

namespace delphyne {
namespace {

class IgnPublisherSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_.Subscribe(
        kTopicName, &IgnPublisherSystemTest::OnTopicMessage, this);
  }

  // Ignition transport topic subscriber callback, for testing
  // purposes.
  void OnTopicMessage(const ignition::msgs::Model_V& message) {
    last_received_ign_msg_ = message;
    received_ign_msg_count_++;
  }

  // Ignition transport node.
  ignition::transport::Node node_{};
  // Count of received ignition messages from ignition
  // transport `kTopicName` topic.
  int received_ign_msg_count_{0};
  // Last received ignition message from ignition
  // transport `kTopicName` topic.
  ignition::msgs::Model_V last_received_ign_msg_{};

  // Ignition transport topic name to subscribe/publish to.
  const std::string kTopicName{"visualizer/scene_update"};
  // Dummy ignition message for testing purposes.
  const ignition::msgs::Model_V kIgnMsg{
    test::BuildPreloadedModelVMsg()};
};

// Creates an Ignition Publisher System and publish a message, then checks that
// it has been correctly received.
TEST_F(IgnPublisherSystemTest, ImmediatePublishTest) {
  const IgnPublisherSystem<ignition::msgs::Model_V> ign_pub(kTopicName);

  drake::systems::Simulator<double> simulator(
      ign_pub, ign_pub.CreateDefaultContext());

  // Configures context's input with the pre-loaded message.
  simulator.get_mutable_context().FixInputPort(
      0, drake::systems::AbstractValue::Make(kIgnMsg));

  // Simulates for a small time period.
  simulator.Initialize();
  const double kPublishDeadline{0.1};
  simulator.StepTo(kPublishDeadline);

  // Introduces a wallclock delay for ignition transport to
  // complete.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Checks that the correct amount of messages have been published.
  ASSERT_EQ(simulator.get_num_steps_taken(), received_ign_msg_count_);

  // Verifies the equivalence of the original ignition message
  // and the received one.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(
      kIgnMsg, last_received_ign_msg_));
}

// Creates an Ignition Publisher System and publish a message repeatedly at a
// low frequency, then checks that it has been received the correct amount of
// times.
TEST_F(IgnPublisherSystemTest, LowFrequencyPublishTest) {
  const double kPublishRateHz = 4.;
  const IgnPublisherSystem<ignition::msgs::Model_V> ign_pub(
      kTopicName, kPublishRateHz);

  drake::systems::Simulator<double> simulator(
      ign_pub, ign_pub.CreateDefaultContext());

  // Configures context's input with the pre-loaded message.
  simulator.get_mutable_context().FixInputPort(
      0, drake::systems::AbstractValue::Make(kIgnMsg));

  // Simulates enough for the expected message count to get
  // published.
  simulator.Initialize();
  const int kMessagesToPublish{5};
  const double kPublishDeadline =
      kMessagesToPublish / kPublishRateHz;
  simulator.StepTo(kPublishDeadline);

  // Introduces a wallclock delay for ignition transport to
  // complete.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Checks that the correct amount of messages have been published.
  ASSERT_EQ(kMessagesToPublish, received_ign_msg_count_);

  // Verifies the equivalence of the original ignition message
  // and the received one.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(
      kIgnMsg, last_received_ign_msg_));
}

}  // namespace
}  // namespace delphyne
