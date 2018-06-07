// Copyright 2018 Toyota Research Institute

#include <memory>

#include <drake/systems/framework/framework_common.h>

#include <gtest/gtest.h>

#include "helpers.h"
#include "translations/drake_simple_car_state_to_ign.h"

namespace delphyne {

// @brief Checks that a Drake simple car state message on the input port is
// correctly translated into an ignition simple car state message.
GTEST_TEST(DrakeSimpleCarStateToIgnTranslatorSystemTest, TestTranslation) {
  const double kExpectedX{1.9};
  const double kExpectedY{2.8};
  const double kExpectedHeading{3.7};
  const double kExpectedVelocity{4.6};

  drake::automotive::SimpleCarState<double> drake_msg;
  drake_msg.set_x(kExpectedX);
  drake_msg.set_y(kExpectedY);
  drake_msg.set_heading(kExpectedHeading);
  drake_msg.set_velocity(kExpectedVelocity);

  const DrakeSimpleCarStateToIgn translator;
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
      output->get_data(kPortIndex)->GetValue<ignition::msgs::SimpleCarState>();

  EXPECT_EQ(ign_msg.x(), kExpectedX);
  EXPECT_EQ(ign_msg.y(), kExpectedY);
  EXPECT_EQ(ign_msg.heading(), kExpectedHeading);
  EXPECT_EQ(ign_msg.velocity(), kExpectedVelocity);
}

}  // namespace delphyne
