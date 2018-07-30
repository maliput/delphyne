// Copyright 2018 Toyota Research Institute

#include "translations/ign_agent_state_to_drake.h"

#include <drake/systems/framework/framework_common.h>

#include <gtest/gtest.h>

namespace delphyne {

// @brief Checks that an ignition simple car state message on the input port is
// correctly translated into a Drake simple car state message.
GTEST_TEST(IgnAgentStateToDrakeTranslatorSystemTest, TestTranslation) {
  const double kExpectedX{1.9};
  const double kExpectedY{2.8};
  const double kExpectedHeading{3.7};
  const double kExpectedVelocity{4.6};

  ignition::msgs::AgentState ign_msg;
  ign_msg.mutable_position()->set_x(kExpectedX);
  ign_msg.mutable_position()->set_y(kExpectedY);
  ign_msg.mutable_orientation()->set_yaw(kExpectedHeading);
  ign_msg.set_velocity(kExpectedVelocity);

  const IgnAgentStateToDrake translator;
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
