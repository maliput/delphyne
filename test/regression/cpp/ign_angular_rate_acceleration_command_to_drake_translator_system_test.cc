// Copyright 2018 Toyota Research Institute

#include "translations/ign_angular_rate_acceleration_command_to_drake.h"

#include <drake/systems/framework/framework_common.h>

#include <gtest/gtest.h>

namespace delphyne {

// @brief Checks that an ignition driving command message on the input port is
// correctly translated into a Drake angular rate / acceleration command message.
GTEST_TEST(IgnAngularRateAccelerationCommandToDrakeTranslatorSystemTest, TestTranslation) {
  const double kOmega{0.12};
  const double kAcceleration{15.7};

  ignition::msgs::AutomotiveAngularRateAccelerationCommand ign_msg;
  ign_msg.set_omega(kOmega);
  ign_msg.set_acceleration(kAcceleration);

  const IgnAngularRateAccelerationCommandToDrake translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex, drake::AbstractValue::Make(ign_msg));

  auto output = translator.AllocateOutput();
  translator.CalcOutput(*context, output.get());

  const auto* vector = output->get_vector_data(kPortIndex);
  const auto drake_msg = dynamic_cast<const AngularRateAccelerationCommand<double>*>(vector);

  EXPECT_EQ(drake_msg->angular_rate(), kOmega);
  EXPECT_EQ(drake_msg->acceleration(), kAcceleration);
}

}  // namespace delphyne
