// Copyright 2017 Toyota Research Institute

#include "backend/discrete_value_to_ignition_message_converter.h"

#include <drake/automotive/gen/simple_car_state.h>
#include <drake/systems/framework/framework_common.h>

#include "gtest/gtest.h"

#include <protobuf/simple_car_state.pb.h>

namespace delphyne {
namespace backend {

using drake::automotive::SimpleCarState;
using drake::automotive::SimpleCarStateIndices;

namespace test {

// Defines a DiscreteValueToIgnitionMessageConverter inherited class that
// implements the
// abstract method vectorToIgn to set a single time.secs value.
class MockedVectorInputToIgnConverter
    : public DiscreteValueToIgnitionMessageConverter<
          ignition::msgs::SimpleCarState, SimpleCarState<double>> {
 public:
  int get_vector_size() { return SimpleCarStateIndices::kNumCoordinates; }

 protected:
  void VectorToIgn(const SimpleCarState<double>&, double,
                   ignition::msgs::SimpleCarState* ign_message) override {
    // Sets a fixed time value to the ign_message.
    ign_message->mutable_time()->set_sec(12345);
  }

  void IgnToVector(const ignition::msgs::SimpleCarState&,
                   SimpleCarState<double>*) override{};
};

// \brief Testing class with common resources for all the tests.
class DiscreteValueToIgnitionMessageConverterTest : public ::testing::Test {
 protected:
  std::unique_ptr<MockedVectorInputToIgnConverter> converter;

  std::unique_ptr<MockedVectorInputToIgnConverter> test_converter;

  std::unique_ptr<IgnPublisherSystem<ignition::msgs::SimpleCarState>>
      ign_publisher;

  const std::string channel{"TEST_CHANNEL"};

 public:
  void SetUp() override {
    // Converter required by the ignition publisher.
    converter = std::make_unique<MockedVectorInputToIgnConverter>();

    // Since the IgnPublisherSystem takes ownership of the converter
    // declared above, we create another one for testing purposes.
    test_converter = std::make_unique<MockedVectorInputToIgnConverter>();

    ign_publisher =
        std::make_unique<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
            channel, std::move(converter));
  }
};

// \brief Asserts that a given input has been processed
// correctly by verifying the value of the resulting ign_msg.
TEST_F(DiscreteValueToIgnitionMessageConverterTest, TestProcessInput) {
  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher->CreateDefaultContext();

  // Defines an empty ignition message.
  auto ign_msg = std::make_unique<ignition::msgs::SimpleCarState>();

  const int kPortIndex(0);

  // Configures context's input with an empty car state message.
  context->FixInputPort(kPortIndex, std::make_unique<SimpleCarState<double>>());

  // Processes the input from the publisher.
  test_converter->ProcessInput(ign_publisher.get(), *context, kPortIndex,
                               ign_msg.get());

  // Asserts that the vector message has been translated.
  EXPECT_EQ(ign_msg->time().sec(), 12345);
}

}  // namespace test
}  // namespace backend
}  // namespace delphyne
