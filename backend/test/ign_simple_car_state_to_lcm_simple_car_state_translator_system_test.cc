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

#include "backend/ign_simple_car_state_to_lcm_simple_car_state_translator_system.h"

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

namespace delphyne {
namespace backend {

// @brief Checks that an ignition simple car state message on the input port is
// correctly translated into an LCM simple car state message.
GTEST_TEST(IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystemTest,
           TestTranslation) {
  const double kExpectedX{1.9};
  const double kExpectedY{2.8};
  const double kExpectedHeading{3.7};
  const double kExpectedVelocity{4.6};

  ignition::msgs::SimpleCarState ign_msg;
  ign_msg.set_x(kExpectedX);
  ign_msg.set_y(kExpectedY);
  ign_msg.set_heading(kExpectedHeading);
  ign_msg.set_velocity(kExpectedVelocity);

  IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystem translator;
  auto context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(ign_msg));

  auto output = translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto* vector = output->get_vector_data(kPortIndex);
  const auto lcm_msg =
      dynamic_cast<const drake::automotive::SimpleCarState<double>*>(vector);

  EXPECT_EQ(lcm_msg->x(), kExpectedX);
  EXPECT_EQ(lcm_msg->y(), kExpectedY);
  EXPECT_EQ(lcm_msg->heading(), kExpectedHeading);
  EXPECT_EQ(lcm_msg->velocity(), kExpectedVelocity);
}

}  // namespace backend
}  // namespace delphyne