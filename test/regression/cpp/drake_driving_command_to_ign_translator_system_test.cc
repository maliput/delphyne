// Copyright 2018 Toyota Research Institute

#include <memory>

#include "drake/systems/framework/framework_common.h"

#include <gtest/gtest.h>

#include "helpers.h"
#include "translations/drake_driving_command_to_ign.h"

namespace delphyne {

// @brief Checks that a Drake driving command message on the input port is
// correctly translated into an ignition driving command message.
GTEST_TEST(DrakeDrivingCommandToIgnTranslatorSystemTest, TestTranslation) {
  const double kTheta{0.12};
  const double kAcceleration{15.7};

  drake::automotive::DrivingCommand<double> drake_msg;
  drake_msg.set_steering_angle(kTheta);
  drake_msg.set_acceleration(kAcceleration);

  const DrakeDrivingCommandToIgn translator;
  std::unique_ptr<drake::systems::Context<double>> context =
      translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(
      kPortIndex,
      std::make_unique<
          drake::systems::Value<drake::systems::BasicVector<double>>>(
          drake_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg =
      output->get_data(kPortIndex)
          ->GetValue<ignition::msgs::AutomotiveDrivingCommand>();

  EXPECT_EQ(ign_msg.theta(), kTheta);
  EXPECT_EQ(ign_msg.acceleration(), kAcceleration);
}

}  // namespace delphyne
