// Copyright 2018 Toyota Research Institute

#include "backend/lcm_simple_car_state_to_ign_simple_car_state_translator_system.h"

#include <memory>

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

// @brief Checks that an LCM simple car state message on the input port is
// correctly translated into an ignition simple car state message.
GTEST_TEST(LcmSimpleCarStateToIgnSimpleCarStateTranslatorSystemTest,
           TestTranslation) {
  const double kExpectedX{1.9};
  const double kExpectedY{2.8};
  const double kExpectedHeading{3.7};
  const double kExpectedVelocity{4.6};

  drake::automotive::SimpleCarState<double> lcm_msg;
  lcm_msg.set_x(kExpectedX);
  lcm_msg.set_y(kExpectedY);
  lcm_msg.set_heading(kExpectedHeading);
  lcm_msg.set_velocity(kExpectedVelocity);

  LcmSimpleCarStateToIgnSimpleCarStateTranslatorSystem translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(
      kPortIndex,
      std::make_unique<
          drake::systems::Value<drake::systems::BasicVector<double>>>(lcm_msg));

  auto output = translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg =
      output->get_data(kPortIndex)->GetValue<ignition::msgs::SimpleCarState>();

  EXPECT_EQ(ign_msg.x(), kExpectedX);
  EXPECT_EQ(ign_msg.y(), kExpectedY);
  EXPECT_EQ(ign_msg.heading(), kExpectedHeading);
  EXPECT_EQ(ign_msg.velocity(), kExpectedVelocity);
}

}  // namespace backend
}  // namespace delphyne
