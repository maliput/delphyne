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

#include "backend/lcm_viewer_draw_to_ignition_message_converter.h"

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
GTEST_TEST(LCMViewerDrawToIgnitionMessageConverterTest, TestProcessInput) {
  // Converter required by the ignition publisher.
  auto converter = std::make_unique<LCMViewerDrawToIgnitionMessageConverter>();

  // Since the IgnPublisherSystem takes ownership of the converter
  // declared above, we create another one for testing purposes.
  auto test_converter =
      std::make_unique<LCMViewerDrawToIgnitionMessageConverter>();

  // IgnitionPublisherSystem takes ownership over the converter.
  auto ign_publisher =
      std::make_unique<IgnPublisherSystem<ignition::msgs::Model_V>>(
          "DRAKE_VIEWER_DRAW", std::move(converter));

  std::unique_ptr<drake::systems::Context<double>> context =
      ign_publisher->CreateDefaultContext();

  const drake::lcmt_viewer_draw lcm_msg{test::BuildPreloadedDrawMsg()};

  // Configures context's input with the pre-loaded message.
  const int kPortIndex{0};
  context->FixInputPort(
      kPortIndex, std::make_unique<drake::systems::Value<drake::lcmt_viewer_draw>>(
             lcm_msg));


  // Calls the ProcessInput method from our test_converter object, since
  // the other converter now belongs to the ign_publisher object.
  auto ign_msg = std::make_unique<ignition::msgs::Model_V>();
  test_converter->ProcessInput(ign_publisher.get(), *context, kPortIndex,
                               ign_msg.get());

  // Check translation's correctness.
  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, *ign_msg));
}

// \brief Checks that an lcm message from the output port is
// correctly translated from its ignition-msgs counterpart.
GTEST_TEST(LCMViewerDrawToIgnitionMessageConverterTest, TestProcessOutput) {
  auto converter = std::make_unique<LCMViewerDrawToIgnitionMessageConverter>();

  const ignition::msgs::Model_V ign_msg{test::BuildPreloadedModelVMsg()};

  auto output_value = converter->AllocateAbstractDefaultValue();
  converter->ProcessAbstractOutput(ign_msg, output_value.get());

  // Check translation's correctness.
  auto lcm_msg = output_value->GetMutableValue<drake::lcmt_viewer_draw>();
  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg));
}

}  // namespace backend
}  // namespace delphyne
