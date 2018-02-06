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

#include "backend/vector_input_to_ign_converter.h"

#include <drake/automotive/gen/simple_car_state.h>
#include <drake/systems/framework/framework_common.h>

#include "gtest/gtest.h"

#include <protobuf/simple_car_state.pb.h>

namespace delphyne {
namespace backend {

using drake::automotive::SimpleCarStateIndices;

namespace test {

// Defines a VectorToIgnConverter inherited class that implements the
// abstract method vectorToIgn to set a single time.secs value.
class MockedVectorInputToIgnConverter
    : public VectorToIgnConverter<ignition::msgs::SimpleCarState> {
 public:
  explicit MockedVectorInputToIgnConverter(int size)
      : VectorToIgnConverter(size) {}

 protected:
  void vectorToIgn(const VectorBase<double>& input_vector, double time,
                   ignition::msgs::SimpleCarState* ign_message) override {
    // Sets a fixed time value to the ign_message.
    ign_message->mutable_time()->set_sec(12345);
  }
};

// \brief Testing class with common resources for all the tests.
class VectorInputToIgnConverterTest : public ::testing::Test {
 protected:
  std::unique_ptr<MockedVectorInputToIgnConverter> converter;

  std::unique_ptr<MockedVectorInputToIgnConverter> test_converter;

  std::unique_ptr<IgnPublisherSystem<ignition::msgs::SimpleCarState>>
      ign_publisher;

  const std::string channel{"TEST_CHANNEL"};

 public:
  void SetUp() override {
    // Converter required by the ignition publisher.
    converter = std::make_unique<MockedVectorInputToIgnConverter>(
        SimpleCarStateIndices::kNumCoordinates);

    // Since the IgnPublisherSystem takes ownership of the converter
    // declared above, we create another one for testing purposes.
    test_converter = std::make_unique<MockedVectorInputToIgnConverter>(
        SimpleCarStateIndices::kNumCoordinates);

    ign_publisher =
        std::make_unique<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
            channel, std::move(converter));
  }
};

// \brief Asserts that a given input has been processed
// correctly by verifying the value of the resulting ign_msg.
TEST_F(VectorInputToIgnConverterTest, TestProcessInput) {
  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher->CreateDefaultContext();

  // Defines an empty ignition message.
  auto ign_msg = std::make_unique<ignition::msgs::SimpleCarState>();

  const int kPortIndex(0);

  // Processes the input from the publisher.
  test_converter->ProcessInput(ign_publisher.get(), *context, kPortIndex,
                               ign_msg.get());

  // Asserts that the vector message has been translated.
  EXPECT_EQ(ign_msg->time().sec(), 12345);
}

// \brief Asserts that a given input has been processed correctly by
// verifying the value of the resulting ign_msg.
TEST_F(VectorInputToIgnConverterTest, TestDeclareInputPort) {
  // There must be a single input port at this point.
  EXPECT_EQ(ign_publisher->get_num_input_ports(), 1);

  // Makes the test_converter add a second abstract input port to the system.
  test_converter->DeclareInputPort(ign_publisher.get());

  // A new input must have been added to the system.
  EXPECT_EQ(ign_publisher->get_num_input_ports(), 2);

  // And it must be of type vector.
  EXPECT_EQ(ign_publisher->get_input_port(1).get_data_type(),
            drake::systems::kVectorValued);
}

}  // namespace test
}  // namespace backend
}  // namespace delphyne
