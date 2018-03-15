// Copyright 2018 Toyota Research Institute

#include "backend/ign_simple_car_state_to_lcm_simple_car_state_translator_system.h"

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

namespace delphyne {
namespace backend {

// @brief Checks that an ignition simple car state message on the input port is
// correctly translated into an LCM simple car state message.
GTEST_TEST(IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystemTest,
           TestTranslation) {
  const double kExpectedX{1.9};
  const double kExpectedY{2.8};
  const double kExpectedHeading{3.7};
  const double kExpectedVelocity{4.6};

  ignition::msgs::SimpleCarState ign_msg;
  ign_msg.set_x(kExpectedX);
  ign_msg.set_y(kExpectedY);
  ign_msg.set_heading(kExpectedHeading);
  ign_msg.set_velocity(kExpectedVelocity);

  IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystem translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(ign_msg));

  auto output = translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto* vector = output->get_vector_data(kPortIndex);
  const auto lcm_msg =
      dynamic_cast<const drake::automotive::SimpleCarState<double>*>(vector);

  EXPECT_EQ(lcm_msg->x(), kExpectedX);
  EXPECT_EQ(lcm_msg->y(), kExpectedY);
  EXPECT_EQ(lcm_msg->heading(), kExpectedHeading);
  EXPECT_EQ(lcm_msg->velocity(), kExpectedVelocity);
}

}  // namespace backend
}  // namespace delphyne
