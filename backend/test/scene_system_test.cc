// Copyright 2018 Open Source Robotics Foundation
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

#include "backend/scene_system.h"
#include "gtest/gtest.h"

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <ignition/msgs.hh>

namespace delphyne {
namespace backend {

// Checks that all the array-iterable values from
// lcmt_viewer_draw matches their ignition counterpart.
void CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                         const ignition::msgs::Scene& scene) {
  for (int i = 0; i < lcm_msg.num_links; i++) {
    // Step 1: Checks there is a corresponding ignition model for the LCM link.
    bool found = false;
    ignition::msgs::Model model;
    for (int j = 0; j < scene.model_size(); ++j) {
      if (scene.model(j).id() == int64_t{lcm_msg.robot_num[i]}) {
        model = scene.model(j);
        found = true;
      }
    }
    ASSERT_TRUE(found);

    // Step 2: Checks there is a corresponding ignition link for the LCM link.
    found = false;
    ignition::msgs::Link link;
    for (int j = 0; j < model.link_size(); ++j) {
      if (model.link(j).name() == lcm_msg.link_name[i]) {
        link = model.link(j);
        found = true;
      }
    }
    ASSERT_TRUE(found);

    // Step 3: Gets the pose and compares the values.
    const ignition::msgs::Pose pose = link.pose();

    EXPECT_EQ(pose.position().x(), lcm_msg.position[i][0]);
    EXPECT_EQ(pose.position().y(), lcm_msg.position[i][1]);
    EXPECT_EQ(pose.position().z(), lcm_msg.position[i][2]);
    EXPECT_EQ(pose.orientation().w(), lcm_msg.quaternion[i][0]);
    EXPECT_EQ(pose.orientation().x(), lcm_msg.quaternion[i][1]);
    EXPECT_EQ(pose.orientation().y(), lcm_msg.quaternion[i][2]);
    EXPECT_EQ(pose.orientation().z(), lcm_msg.quaternion[i][3]);
  }
}

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

  drake::lcmt_viewer_draw get_preloaded_draw_msg() {
    drake::lcmt_viewer_draw msg;
    msg.timestamp = 0;
    msg.num_links = 1;
    msg.link_name.resize(msg.num_links);
    msg.link_name[0] = "box";
    msg.robot_num.resize(1);
    msg.robot_num[0] = 1;
    msg.position.resize(1);
    msg.position[0].resize(3);
    msg.position[0][0] = 0.0;
    msg.position[0][1] = 0.0;
    msg.position[0][2] = 0.0;
    msg.quaternion.resize(1);
    msg.quaternion[0].resize(4);
    msg.quaternion[0][0] = 0.0;
    msg.quaternion[0][1] = 0.0;
    msg.quaternion[0][2] = 0.0;
    msg.quaternion[0][3] = 1.0;
    return msg;
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
  auto lcm_msg = get_preloaded_draw_msg();

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      0, std::make_unique<drake::systems::Value<drake::lcmt_viewer_draw>>(
             lcm_msg));

  // Makes the IgnPublisherSystem to publish the message.
  scene_publisher_->Publish(*context.get());

  // Checks that ignition-transport topic callback has been called.
  ASSERT_EQ(handler_called_, true);

  // Verifies the equivalence of the original lcm message and
  // the received ignition-transport message.
  CheckMsgTranslation(lcm_msg, scene_msg_);
}

}  // namespace backend
}  // namespace delphyne