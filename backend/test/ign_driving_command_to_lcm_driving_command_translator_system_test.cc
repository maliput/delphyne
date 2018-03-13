// Copyright 2018 Toyota Research Institute

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
