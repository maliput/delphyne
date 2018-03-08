// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"
#include "backend/test/helpers.h"

#include "gtest/gtest.h"

#include <chrono>
#include <thread>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <ignition/msgs.hh>

namespace delphyne {
namespace backend {

class SceneSystemTest : public ::testing::Test {
 public:
  // Ignition transport node.
  ignition::transport::Node node_;
  // Callback flag.
  bool handler_called_ = false;
  // The received message.
  ignition::msgs::Scene scene_msg_;
  // Scene System pointer.
  std::unique_ptr<SceneSystem> scene_publisher_ =
      std::make_unique<SceneSystem>("/scene");

  void SetUp() override {
    // Register callback.
    this->node_.Subscribe(scene_publisher_->get_topic_name(),
                          &SceneSystemTest::SubscriberMockCallback, this);

    handler_called_ = false;
  }

 private:
  void SubscriberMockCallback(const ignition::msgs::Scene& message) {
    scene_msg_ = message;
    handler_called_ = true;
  }
};

// Creates a Scene System and publish a
// message, then checks that it has been correctly received.
TEST_F(SceneSystemTest, PublishTest) {
  std::unique_ptr<drake::systems::Context<double>> context =
      scene_publisher_->CreateDefaultContext();

  // Fills the lcm message with sample data.
  const auto lcm_msg = test::BuildPreloadedDrawMsg();

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      0, std::make_unique<drake::systems::Value<drake::lcmt_viewer_draw>>(
             lcm_msg));

  // Makes the IgnPublisherSystem to publish the message.
  scene_publisher_->Publish(*context.get());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Checks that ignition-transport topic callback has been called.
  ASSERT_EQ(handler_called_, true);

  // Verifies the equivalence of the original lcm message and
  // the received ignition-transport message.
  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, scene_msg_));
}

}  // namespace backend
}  // namespace delphyne
