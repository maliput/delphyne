// Copyright 2018 Toyota Research Institute

#include "backend/lcm_driving_command_to_ign_driving_command_translator_system.h"

#include <memory>

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

// @brief Checks that an LCM driving command message on the input port is
// correctly
//        translated into an ignition driving command message.
GTEST_TEST(LcmDrivingCommandToIgnDrivingCommandTranslatorSystemTest,
           TestTranslation) {
  const double kTheta{0.12};
  const double kAcceleration{15.7};

  drake::automotive::DrivingCommand<double> lcm_msg;
  lcm_msg.set_steering_angle(kTheta);
  lcm_msg.set_acceleration(kAcceleration);

  LcmDrivingCommandToIgnDrivingCommandTranslatorSystem translator;
  auto context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(
      kPortIndex,
      std::make_unique<
          drake::systems::Value<drake::systems::BasicVector<double>>>(lcm_msg));

  auto output = translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg =
      output->get_data(kPortIndex)
          ->GetValue<ignition::msgs::AutomotiveDrivingCommand>();

  EXPECT_EQ(ign_msg.theta(), kTheta);
  EXPECT_EQ(ign_msg.acceleration(), kAcceleration);
}

}  // namespace backend
}  // namespace delphyne
