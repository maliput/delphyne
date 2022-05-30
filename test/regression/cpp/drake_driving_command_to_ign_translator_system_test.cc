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

#include <memory>

#include <drake/systems/framework/framework_common.h>
#include <gtest/gtest.h>

#include "test_utilities/helpers.h"
#include "translations/drake_driving_command_to_ign.h"

namespace delphyne {

// @brief Checks that a Drake driving command message on the input port is
// correctly translated into an ignition driving command message.
GTEST_TEST(DrakeDrivingCommandToIgnTranslatorSystemTest, TestTranslation) {
  const double kTheta{0.12};
  const double kAcceleration{15.7};

  DrivingCommand<double> drake_msg;
  drake_msg.set_steering_angle(kTheta);
  drake_msg.set_acceleration(kAcceleration);

  const DrakeDrivingCommandToIgn translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex, drake::Value<drake::systems::BasicVector<double>>(drake_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output = translator.AllocateOutput();
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg = output->get_data(kPortIndex)->get_value<ignition::msgs::AutomotiveDrivingCommand>();

  EXPECT_EQ(ign_msg.theta(), kTheta);
  EXPECT_EQ(ign_msg.acceleration(), kAcceleration);
}

}  // namespace delphyne
