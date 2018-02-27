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

#include "backend/driving_command_to_ignition_message_converter.h"

#include "drake/automotive/gen/driving_command.h"
#include "gtest/gtest.h"
#include "ignition/msgs.hh"
#include "protobuf/automotive_driving_command.pb.h"

namespace delphyne {
namespace backend {
namespace test {

GTEST_TEST(DrivingCommandToIgnitionMessageConverter,
           TestAutomotiveDrivingCommandTranslation) {
  ignition::msgs::AutomotiveDrivingCommand ign_driving_message;

  const double kTheta = 0.12;
  const double kAcceleration = 15.7;

  ign_driving_message.set_theta(kTheta);
  ign_driving_message.set_acceleration(kAcceleration);

  DrivingCommandToIgnitionMessageConverter converter;
  std::unique_ptr<BasicVector<double>> output_vector =
      converter.AllocateDiscreteOutputValue();

  converter.ProcessDiscreteOutput(ign_driving_message, output_vector.get());

  auto* const driving_command_vector =
      dynamic_cast<DrivingCommand<double>*>(output_vector.get());

  EXPECT_EQ(kTheta, driving_command_vector->steering_angle());
  EXPECT_EQ(kAcceleration, driving_command_vector->acceleration());
}

}  // namespace test
}  // namespace backend
}  // namespace delphyne
