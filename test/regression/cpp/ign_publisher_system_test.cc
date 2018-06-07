// Copyright 2018 Toyota Research Institute

#include "backend/ign_publisher_system.h"

#include <chrono>
#include <string>
#include <thread>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include <gtest/gtest.h>

#include <ignition/msgs.hh>

#include "helpers.h"

namespace delphyne {

class IgnPublisherSystemTest : public ::testing::Test {
  void SubscriberMockCallback(const ignition::msgs::Model_V& message) {
    ign_msg_ = message;
    handler_called_count_++;
  }

 public:
  // Ignition transport node.
  ignition::transport::Node node_;

  // Callback count.
  int handler_called_count_ = -1;

  // The received message.
  ignition::msgs::Model_V ign_msg_;

  const std::string kTopicName = "visualizer/scene_update";

  void SetUp() override {
    handler_called_count_ = 0;

    node_.Subscribe(kTopicName, &IgnPublisherSystemTest::SubscriberMockCallback,
                    this);
  }
};

// Creates an Ignition Publisher System and publish a message, then checks that
// it has been correctly received.
TEST_F(IgnPublisherSystemTest, PublishTest) {
  IgnPublisherSystem<ignition::msgs::Model_V> ign_publisher(kTopicName);

  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher.CreateDefaultContext();

  // Fills the ign message with sample data.
  const ignition::msgs::Model_V ign_msg = test::BuildPreloadedModelVMsg();

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      0, std::make_unique<drake::systems::Value<ignition::msgs::Model_V>>(
             ign_msg));

  // Makes the IgnPublisherSystem to publish the message.
  ign_publisher.Publish(*context.get());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Checks that ignition-transport topic callback has been called.
  ASSERT_EQ(1, handler_called_count_);

  // Verifies the equivalence of the original lcm message and
  // the received ignition-transport message.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(ign_msg, ign_msg_));
}

// Creates an Ignition Publisher System and publish a message repeatedly at a
// low frequency, then checks that it has been received the correct amount of
// times.
TEST_F(IgnPublisherSystemTest, LowFrequencyPublishTest) {
  const double kPublishPeriodMs = 250.0;
  IgnPublisherSystem<ignition::msgs::Model_V> ign_publisher(kTopicName,
                                                            kPublishPeriodMs);

  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher.CreateDefaultContext();

  // Fills the ign message with sample data.
  const ignition::msgs::Model_V ign_msg = test::BuildPreloadedModelVMsg();

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      0, std::make_unique<drake::systems::Value<ignition::msgs::Model_V>>(
             ign_msg));

  const int kMessagesToPublish = 4;
  // The first message is published immediately, so we need to wait a little bit
  // more than the period times the number of messages, minus one.
  const std::chrono::duration<double, std::milli> kPublishingTime{
      kPublishPeriodMs * 1.1 * (kMessagesToPublish - 1)};

  // Since the Drake simulator is not running, we need to call Publish on our
  // own, simulating the simulator.
  const std::chrono::time_point<std::chrono::steady_clock> start =
      std::chrono::steady_clock::now();
  auto now = start;

  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ign_publisher.Publish(*context.get());

    now = std::chrono::steady_clock::now();
  } while ((now - start) < kPublishingTime);

  // Checks the correct amount of messages have been published.
  ASSERT_EQ(kMessagesToPublish, handler_called_count_);
}

}  // namespace delphyne
