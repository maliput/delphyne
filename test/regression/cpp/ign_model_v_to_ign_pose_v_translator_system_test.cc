// Copyright 2018 Toyota Research Institute

#include "translations/ign_model_v_to_ign_pose_v.h"

#include <drake/systems/framework/framework_common.h>

#include <gtest/gtest.h>

#include "test_utilities/helpers.h"

namespace delphyne {

// @brief Checks that an ignition Model_V message on the input port is correctly
// translated into an ignition Pose_V message.
GTEST_TEST(IgnModelVToIgnPoseVTranslatorSystemTest, TestTranslation) {
  const ignition::msgs::Model_V ign_msg{test::BuildPreloadedModelVMsg()};

  const IgnModelVToIgnPoseV translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex, drake::Value<ignition::msgs::Model_V>(ign_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output = translator.AllocateOutput();
  translator.CalcOutput(*context, output.get());

  const auto& pose_v = output->get_data(kPortIndex)->get_value<ignition::msgs::Pose_V>();

  EXPECT_TRUE(test::CheckMsgTranslation(ign_msg, pose_v));
}

}  // namespace delphyne
