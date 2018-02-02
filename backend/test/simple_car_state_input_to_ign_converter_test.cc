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

#include "backend/simple_car_state_input_to_ign_converter.h"
#include "backend/test/helpers.h"

#include <drake/automotive/gen/simple_car_state.h>
#include <drake/automotive/gen/simple_car_state_translator.h>

#include "gtest/gtest.h"

#include <protobuf/simple_car_state.pb.h>

namespace delphyne {
namespace backend {
namespace test {

// \brief Checks that a vector input of type SimpleCarState is correctly
// processed and translated into its ignition-msgs counterpart.
GTEST_TEST(SimpleCarStateInputToIgnConverterTest, TestVectorToIgn) {
  // The size of the vector.
  const int kSize = 1;

  // Converter required by the ignition publisher.
  auto converter = std::make_unique<SimpleCarStateInputToIgnConverter>(kSize);

  // Since the IgnPublisherSystem takes ownership of the converter
  // declared above, it creates another one for testing purposes.
  auto test_converter =
      std::make_unique<SimpleCarStateInputToIgnConverter>(kSize);

  // IgnitionPublisherSystem takes ownership over the converter.
  auto ign_publisher =
      std::make_unique<IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
          "TEST_CHANNEL", std::move(converter));

  test_converter->DeclareInputPort(ign_publisher.get());

  // Creates a context into the publisher.
  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher->CreateDefaultContext();

  // Sets time value, since it'll be included as part of the
  // ignition message in the translation.
  context->set_time(123.456);

  // Loads the SimpleCarState message with constant values.
  drake::automotive::SimpleCarState<double> car_state;
  car_state.set_x(1.9);
  car_state.set_y(2.8);
  car_state.set_heading(3.7);
  car_state.set_velocity(4.6);

  const int kPortIndex{0};

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
  EXPECT_EQ(ign_msg->x(), 1.9);
  EXPECT_EQ(ign_msg->y(), 2.8);
  EXPECT_EQ(ign_msg->heading(), 3.7);
  EXPECT_EQ(ign_msg->velocity(), 4.6);
}

}  // namespace test
}  // namespace backend
}  // namespace delphyne
