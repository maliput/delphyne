// Copyright 2018 Toyota Research Institute

#include "backend/translation_systems/ign_simple_car_state_to_drake.h"

#include "drake/systems/framework/framework_common.h"

#include <gtest/gtest.h>

namespace delphyne {

// @brief Checks that an ignition simple car state message on the input port is
// correctly translated into a Drake simple car state message.
GTEST_TEST(IgnSimpleCarStateToDrakeTranslatorSystemTest, TestTranslation) {
  const double kExpectedX{1.9};
  const double kExpectedY{2.8};
  const double kExpectedHeading{3.7};
  const double kExpectedVelocity{4.6};

  ignition::msgs::SimpleCarState ign_msg;
  ign_msg.set_x(kExpectedX);
  ign_msg.set_y(kExpectedY);
  ign_msg.set_heading(kExpectedHeading);
  ign_msg.set_velocity(kExpectedVelocity);

  const translation_systems::IgnSimpleCarStateToDrake translator;
  std::unique_ptr<drake::systems::Context<double>> context =
      translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(ign_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const drake::systems::BasicVector<double>* vector =
      output->get_vector_data(kPortIndex);
  const auto drake_msg =
      dynamic_cast<const drake::automotive::SimpleCarState<double>*>(vector);

  EXPECT_EQ(drake_msg->x(), kExpectedX);
  EXPECT_EQ(drake_msg->y(), kExpectedY);
  EXPECT_EQ(drake_msg->heading(), kExpectedHeading);
  EXPECT_EQ(drake_msg->velocity(), kExpectedVelocity);
}

}  // namespace delphyne
