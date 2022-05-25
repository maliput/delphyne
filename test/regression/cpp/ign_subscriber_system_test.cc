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

#include "backend/ign_subscriber_system.h"

#include <chrono>
#include <thread>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <gtest/gtest.h>
#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "test_utilities/helpers.h"

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
  ignition::transport::Node::Publisher publisher = node.Advertise<ignition::msgs::Model_V>(kTopicName);

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

  std::unique_ptr<drake::systems::Context<double>> context = subscriber_system.AllocateContext();
  std::unique_ptr<drake::systems::CompositeEventCollection<double>> event_info =
      subscriber_system.AllocateCompositeEventCollection();

  // This call should make the system create a new update event.
  subscriber_system.CalcNextUpdateTime(*context, event_info.get());
  ASSERT_TRUE(event_info->HasEvents());
  ASSERT_TRUE(event_info->HasUnrestrictedUpdateEvents());

  // The update is performed over a temporary state, which is then copied to the
  // real context.
  std::unique_ptr<drake::systems::State<double>> tmp_state = context->CloneState();
  subscriber_system.CalcUnrestrictedUpdate(*context, event_info->get_unrestricted_update_events(), tmp_state.get());
  context->get_mutable_state().SetFrom(*tmp_state);

  // Finally, the context containing the results of the update can be used to
  // calculate the system's output.
  std::unique_ptr<drake::systems::SystemOutput<double>> output = subscriber_system.AllocateOutput();
  subscriber_system.CalcOutput(*context, output.get());

  const auto& received_message = output->get_data(0)->get_value<ignition::msgs::Model_V>();

  // Verifies the received message matches the published message.
  EXPECT_TRUE(test::CheckProtobufMsgEquality(ign_message, received_message));
}

}  // namespace delphyne
