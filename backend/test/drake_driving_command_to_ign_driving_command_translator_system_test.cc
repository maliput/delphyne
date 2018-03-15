// Copyright 2018 Toyota Research Institute

#include "backend/drake_driving_command_to_ign_driving_command_translator_system.h"

#include <memory>

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

// @brief Checks that a Drake driving command message on the input port is
// correctly translated into an ignition driving command message.
GTEST_TEST(DrakeDrivingCommandToIgnDrivingCommandTranslatorSystemTest,
           TestTranslation) {
  const double kTheta{0.12};
  const double kAcceleration{15.7};

  drake::automotive::DrivingCommand<double> drake_msg;
  drake_msg.set_steering_angle(kTheta);
  drake_msg.set_acceleration(kAcceleration);

  translation_systems::DrakeDrivingCommandToIgnDrivingCommand translator;
  std::unique_ptr<drake::systems::Context<double>> context =
      translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(
      kPortIndex,
      std::make_unique<
          drake::systems::Value<drake::systems::BasicVector<double>>>(
          drake_msg));

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
