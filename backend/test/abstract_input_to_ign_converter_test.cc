// Copyright 2017 Open Source Robotics Foundation
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

#include "backend/abstract_input_to_ign_converter.h"
#include "backend/ign_publisher_system.h"
#include "backend/test/helpers.cc"

#include "gtest/gtest.h"

#include <drake/lcmt_viewer_draw.hpp>

#include <ignition/msgs.hh>

namespace delphyne {
namespace backend {

// \brief Checks that an lcm message from the input port is
// correctly translated into its ignition-msgs counterpart.
GTEST_TEST(AbstractInputToIgnConverterTest, TestProcessInputWithValidTypes) {
  // Converter required by the ignition publisher.
  auto converter =
      std::make_unique<AbstractInputToIgnConverter<drake::lcmt_viewer_draw,
                                                   ignition::msgs::Model_V>>();

  // Since the IgnPublisherSystem takes ownership of the converter
  // declared above, we create another one for testing purposes.
  auto test_converter =
      std::make_unique<AbstractInputToIgnConverter<drake::lcmt_viewer_draw,
                                                   ignition::msgs::Model_V>>();

  // IgnitionPublisherSystem takes ownership over the converter.
  auto ign_publisher =
      std::make_unique<IgnPublisherSystem<ignition::msgs::Model_V>>(
          "DRAKE_VIEWER_DRAW", std::move(converter));

  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher->CreateDefaultContext();

  auto lcm_msg{get_preloaded_draw_msg()};

  auto ign_msg{std::make_unique<ignition::msgs::Model_V>()};

  // Configures context's input with the pre-loaded message.
  context->FixInputPort(
      0, std::make_unique<drake::systems::Value<drake::lcmt_viewer_draw>>(
             lcm_msg));

  int kPortIndex{0};

  // Calls the ProcessInput method from our test_converter object, since
  // the other converter now belongs to the ign_publisher object.
  test_converter->ProcessInput(ign_publisher.get(), *context, kPortIndex,
                               ign_msg.get());

  // Check translation's correctness.
  CheckMsgTranslation(lcm_msg, *ign_msg);
}

// \brief Checks that the DeclareInputPort methods
// adds another input port to the IgnPublisherSystem.
GTEST_TEST(AbstractInputToIgnConverterTest, TestProcessInputWithInvalidTypes) {
  // Converter required by the ignition publisher.
  auto converter =
      std::make_unique<AbstractInputToIgnConverter<drake::lcmt_viewer_draw,
                                                   ignition::msgs::Model_V>>();

  // Since the IgnPublisherSystem takes ownership of the converter
  // declared above, we create another one for testing purposes.
  auto test_converter =
      std::make_unique<AbstractInputToIgnConverter<drake::lcmt_viewer_draw,
                                                   ignition::msgs::Model_V>>();

  // Ignition Publisher System takes ownership over the converter.
  auto ign_publisher =
      std::make_unique<IgnPublisherSystem<ignition::msgs::Model_V>>(
          "DRAKE_VIEWER_DRAW", std::move(converter));

  // We expect a single abstract input, added during the constructor call.
  EXPECT_EQ(ign_publisher->get_num_input_ports(), 1);

  // Make the test_converter add a second abstract input port to the system.
  test_converter->DeclareInputPort(ign_publisher.get());

  // We should have two abstract inputs now.
  EXPECT_EQ(ign_publisher->get_num_input_ports(), 2);
}

}  // namespace backend
}  // namespace delphyne
