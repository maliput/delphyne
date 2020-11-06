// Copyright 2018 Toyota Research Institute

#include <memory>

#include <drake/systems/framework/framework_common.h>

#include <gtest/gtest.h>

#include "test_utilities/helpers.h"
#include "translations/drake_angular_rate_acceleration_command_to_ign.h"

namespace delphyne {

// @brief Checks that a Drake driving command message on the input port is
// correctly translated into an ignition angular rate / acceleration command message.
GTEST_TEST(DrakeAngularRateAccelerationCommandToIgnTranslatorSystemTest, TestTranslation) {
  const double kOmega{0.12};
  const double kAcceleration{15.7};

  AngularRateAccelerationCommand<double> drake_msg;
  drake_msg.set_angular_rate(kOmega);
  drake_msg.set_acceleration(kAcceleration);

  const DrakeAngularRateAccelerationCommandToIgn translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex, std::make_unique<drake::Value<drake::systems::BasicVector<double>>>(drake_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output = translator.AllocateOutput();
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg =
      output->get_data(kPortIndex)->get_value<ignition::msgs::AutomotiveAngularRateAccelerationCommand>();

  EXPECT_EQ(ign_msg.omega(), kOmega);
  EXPECT_EQ(ign_msg.acceleration(), kAcceleration);
}

}  // namespace delphyne
