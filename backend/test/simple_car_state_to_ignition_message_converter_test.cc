// Copyright 2017 Toyota Research Institute

#include "backend/simple_car_state_to_ignition_message_converter.h"

#include <drake/automotive/gen/simple_car_state.h>
#include <drake/automotive/gen/simple_car_state_translator.h>

#include "gtest/gtest.h"

#include <protobuf/simple_car_state.pb.h>

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {
namespace test {

// \brief Checks that a vector input of type SimpleCarState is correctly
// processed and translated into its ignition-msgs counterpart.
GTEST_TEST(SimpleCarStateToIgnitionMessageConverterTest, TestVectorToIgn) {
  // Converter required by the ignition publisher.
  auto converter = std::make_unique<SimpleCarStateToIgnitionMessageConverter>();

  // Since the IgnPublisherSystem takes ownership of the converter
  // declared above, it creates another one for testing purposes.
  auto test_converter =
      std::make_unique<SimpleCarStateToIgnitionMessageConverter>();

  // IgnitionPublisherSystem takes ownership over the converter.
  auto ign_publisher =
      std::make_unique<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
          "TEST_CHANNEL", std::move(converter));

  // Creates a context into the publisher.
  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher->CreateDefaultContext();

  // Values used to check against.
  const int kPortIndex{0};
  const double kTimeSecs{123.456};
  const double kExpectedX{1.9};
  const double kExpectedY{2.8};
  const double kExpectedHeading{3.7};
  const double kExpectedVelocity{4.6};

  // Sets time value, since it'll be included as part of the
  // ignition message in the translation.
  context->set_time(kTimeSecs);

  // Loads the SimpleCarState message with constant values.
  drake::automotive::SimpleCarState<double> car_state;
  car_state.set_x(kExpectedX);
  car_state.set_y(kExpectedY);
  car_state.set_heading(kExpectedHeading);
  car_state.set_velocity(kExpectedVelocity);

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      kPortIndex,
      std::make_unique<
          drake::systems::Value<drake::systems::BasicVector<double>>>(
          car_state));

  // Calls the ProcessInput method from our test_converter object, since
  // the other converter now belongs to the ign_publisher object.
  auto ign_msg = std::make_unique<ignition::msgs::SimpleCarState>();

  // The ProcessInput method calls the vectorToIgn method that is
  // currently being tested indirectly.
  test_converter->ProcessInput(ign_publisher.get(), *context, kPortIndex,
                               ign_msg.get());

  // Asserts that the vector message has been translated correctly.
  EXPECT_EQ(ign_msg->time().sec(), 123);
  EXPECT_EQ(ign_msg->time().nsec(), 456000000);
  EXPECT_EQ(ign_msg->x(), kExpectedX);
  EXPECT_EQ(ign_msg->y(), kExpectedY);
  EXPECT_EQ(ign_msg->heading(), kExpectedHeading);
  EXPECT_EQ(ign_msg->velocity(), kExpectedVelocity);
}

}  // namespace test
}  // namespace backend
}  // namespace delphyne
