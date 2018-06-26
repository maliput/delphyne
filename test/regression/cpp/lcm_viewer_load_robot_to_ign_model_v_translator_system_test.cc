// Copyright 2018 Toyota Research Institute

#include "translations/lcm_viewer_load_robot_to_ign_model_v.h"

#include <drake/systems/framework/framework_common.h>

#include <gtest/gtest.h>

#include "helpers.h"

namespace delphyne {

// @brief Checks that an LCM viewer load robot message on the input port is
// correctly translated into an ignition Model_V message.
GTEST_TEST(LCMViewerLoadRobotToIgnModelVTranslatorSystemTest, TestTranslation) {
  const drake::lcmt_viewer_load_robot lcm_msg{
      test::BuildPreloadedLoadRobotMsg()};

  const LcmViewerLoadRobotToIgnModelV translator;
  std::unique_ptr<drake::systems::Context<double>> context =
      translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(lcm_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg =
      output->get_data(kPortIndex)->GetValue<ignition::msgs::Model_V>();

  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg));
}

}  // namespace delphyne
