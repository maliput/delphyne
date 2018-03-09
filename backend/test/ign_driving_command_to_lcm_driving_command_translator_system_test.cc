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

#include "backend/ign_driving_command_to_lcm_driving_command_translator_system.h"

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

namespace delphyne {
namespace backend {

// @brief Checks that an ignition driving command message on the input port is
// correctly
//        translated into an LCM driving command message.
GTEST_TEST(IgnDrivingCommandToLcmDrivingCommandTranslatorSystemTest,
           TestTranslation) {
  const double kTheta{0.12};
  const double kAcceleration{15.7};

  ignition::msgs::AutomotiveDrivingCommand ign_msg;
  ign_msg.set_theta(kTheta);
  ign_msg.set_acceleration(kAcceleration);

  IgnDrivingCommandToLcmDrivingCommandTranslatorSystem translator;
  auto context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(ign_msg));

  auto output = translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto* vector = output->get_vector_data(kPortIndex);
  const auto lcm_msg =
      dynamic_cast<const drake::automotive::DrivingCommand<double>*>(vector);

  EXPECT_EQ(lcm_msg->steering_angle(), kTheta);
  EXPECT_EQ(lcm_msg->acceleration(), kAcceleration);
}

}  // namespace backend
}  // namespace delphyne
