// Copyright 2018 Toyota Research Institute

#include "translations/ign_driving_command_to_drake.h"

#include <drake/systems/framework/framework_common.h>

#include <gtest/gtest.h>

namespace delphyne {

// @brief Checks that an ignition driving command message on the input port is
// correctly translated into a Drake driving command message.
GTEST_TEST(IgnDrivingCommandToDrakeTranslatorSystemTest, TestTranslation) {
  const double kTheta{0.12};
  const double kAcceleration{15.7};

  ignition::msgs::AutomotiveDrivingCommand ign_msg;
  ign_msg.set_theta(kTheta);
  ign_msg.set_acceleration(kAcceleration);

  const IgnDrivingCommandToDrake translator;
  std::unique_ptr<drake::systems::Context<double>> context =
      translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(ign_msg));

  auto output = translator.AllocateOutput();
  translator.CalcOutput(*context, output.get());

  const auto* vector = output->get_vector_data(kPortIndex);
  const auto drake_msg =
      dynamic_cast<const drake::automotive::DrivingCommand<double>*>(vector);

  EXPECT_EQ(drake_msg->steering_angle(), kTheta);
  EXPECT_EQ(drake_msg->acceleration(), kAcceleration);
}

}  // namespace delphyne
