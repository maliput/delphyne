// Copyright 2018 Toyota Research Institute

#include "backend/ign_subscriber_system.h"

#include <chrono>
#include <thread>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

#include <gtest/gtest.h>

#include "ignition/msgs.hh"

#include "backend/system.h"
#include "helpers.h"

namespace delphyne {

// Creates an Ignition Subscriber System and publish a message, then checks that
// it has been correctly received by the system.
GTEST_TEST(IgnSubscriberSystemTest, SubscribeTest) {
  const std::string kTopicName{"visualizer/scene_update"};

  // Subscriber system.
  IgnSubscriberSystem<ignition::msgs::Model_V> subscriber_system(kTopicName);

  // Ignition transport node.
  ignition::transport::Node node;

  // Ignition transport publisher.
  ignition::transport::Node::Publisher publisher =
      node.Advertise<ignition::msgs::Model_V>(kTopicName);

  // Wait for both the subscriber and publisher to be connected before doing the
  // actual publishing.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Fills the ign message with sample data, and publishes it.
  const ignition::msgs::Model_V ign_message = test::BuildPreloadedModelVMsg();
  publisher.Publish(ign_message);

  // Wait for the published message to reach the subscriber.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // To trigger the subscriber system's sync mechanism, we need to have it
  // trigger and process an unrestricted update event.

  std::unique_ptr<drake::systems::Context<double>> context =
      subscriber_system.AllocateContext();
  std::unique_ptr<drake::systems::CompositeEventCollection<double>> event_info =
      subscriber_system.AllocateCompositeEventCollection();

  // This call should make the system create a new update event.
  subscriber_system.CalcNextUpdateTime(*context, event_info.get());
  ASSERT_TRUE(event_info->HasEvents());
  ASSERT_TRUE(event_info->HasUnrestrictedUpdateEvents());

  // The update is performed over a temporary state, which is then copied to the
  // real context.
  std::unique_ptr<drake::systems::State<double>> tmp_state =
      context->CloneState();
  subscriber_system.CalcUnrestrictedUpdate(
      *context, event_info->get_unrestricted_update_events(), tmp_state.get());
  context->get_mutable_state().CopyFrom(*tmp_state);

  // Finally, the context containing the results of the update can be used to
  // calculate the system's output.
  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      subscriber_system.AllocateOutput(*context);
  subscriber_system.CalcOutput(*context, output.get());

  const auto& received_message =
      output->get_data(0)->GetValue<ignition::msgs::Model_V>();

  // Verifies the received message matches the published message.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(ign_message, received_message));
}

}  // namespace delphyne
