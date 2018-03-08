// Copyright 2017 Toyota Research Institute

#include "backend/abstract_value_to_ignition_message_converter.h"

#include <drake/lcmt_viewer_draw.hpp>
#include <drake/systems/framework/framework_common.h>

#include "gtest/gtest.h"

#include <ignition/msgs.hh>

#include "backend/ign_publisher_system.h"
#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

// \brief Checks that an lcm message from the input port is
// correctly translated into its ignition-msgs counterpart.
GTEST_TEST(AbstractValueToIgnitionMessageConverterTest, TestProcessInput) {
  // Converter required by the ignition publisher.
  auto converter = std::make_unique<AbstractValueToIgnitionMessageConverter<
      drake::lcmt_viewer_draw, ignition::msgs::Model_V>>();

  // Since the IgnPublisherSystem takes ownership of the converter
  // declared above, we create another one for testing purposes.
  auto test_converter =
      std::make_unique<AbstractValueToIgnitionMessageConverter<
          drake::lcmt_viewer_draw, ignition::msgs::Model_V>>();

  // IgnitionPublisherSystem takes ownership over the converter.
  auto ign_publisher =
      std::make_unique<IgnPublisherSystem<ignition::msgs::Model_V>>(
          "DRAKE_VIEWER_DRAW", std::move(converter));

  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher->CreateDefaultContext();

  const drake::lcmt_viewer_draw lcm_msg{test::BuildPreloadedDrawMsg()};

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      0, std::make_unique<drake::systems::Value<drake::lcmt_viewer_draw>>(
             lcm_msg));

  const int kPortIndex{0};

  // Calls the ProcessInput method from our test_converter object, since
  // the other converter now belongs to the ign_publisher object.
  auto ign_msg = std::make_unique<ignition::msgs::Model_V>();
  test_converter->ProcessInput(ign_publisher.get(), *context, kPortIndex,
                               ign_msg.get());

  // Check translation's correctness.
  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, *ign_msg));
}

}  // namespace backend
}  // namespace delphyne
