// Copyright 2017 Toyota Research Institute

#include "backend/ign_publisher_system.h"
#include "backend/abstract_value_to_ignition_message_converter.h"
#include "backend/test/helpers.h"

#include "gtest/gtest.h"

#include <chrono>
#include <thread>

#include <drake/lcmt_viewer_draw.hpp>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include <ignition/msgs.hh>

namespace delphyne {
namespace backend {

class IgnPublisherSystemTest : public ::testing::Test {
  void SubscriberMockCallback(const ignition::msgs::Model_V& message) {
    ign_msg_ = message;
    handler_called_ = true;
  }

 public:
  // Ignition transport node.
  ignition::transport::Node node_;

  // Callback flag.
  bool handler_called_ = false;

  // The received message.
  ignition::msgs::Model_V ign_msg_;

  // Converter required by the ignition publisher
  std::unique_ptr<IgnitionMessageConverter<ignition::msgs::Model_V>> converter_{
      std::make_unique<AbstractValueToIgnitionMessageConverter<
          drake::lcmt_viewer_draw, ignition::msgs::Model_V>>()};

  // Ignition Publisher System pointer.
  std::unique_ptr<IgnPublisherSystem<ignition::msgs::Model_V>> ign_publisher_{
      std::make_unique<IgnPublisherSystem<ignition::msgs::Model_V>>(
          "DRAKE_VIEWER_DRAW", std::move(converter_))};

  void SetUp() override {
    // Register callback.
    this->node_.Subscribe(ign_publisher_->get_topic_name(),
                          &IgnPublisherSystemTest::SubscriberMockCallback,
                          this);

    handler_called_ = false;
  }
};

// Creates an Ignition Publisher System and publish a
// message, then checks that it has been correctly received.
TEST_F(IgnPublisherSystemTest, PublishTest) {
  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher_->CreateDefaultContext();

  // Fills the lcm message with sample data.
  const auto lcm_msg = test::BuildPreloadedDrawMsg();

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      0, std::make_unique<drake::systems::Value<drake::lcmt_viewer_draw>>(
             lcm_msg));

  // Makes the IgnPublisherSystem to publish the message.
  ign_publisher_->Publish(*context.get());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Checks that ignition-transport topic callback has been called.
  ASSERT_EQ(handler_called_, true);

  // Verifies the equivalence of the original lcm message and
  // the received ignition-transport message.
  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg_));
}

}  // namespace backend
}  // namespace delphyne
