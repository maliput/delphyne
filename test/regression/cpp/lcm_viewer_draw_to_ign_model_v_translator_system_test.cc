// Copyright 2018 Toyota Research Institute

#include <drake/systems/framework/framework_common.h>
#include <gtest/gtest.h>

#include "test_utilities/helpers.h"
#include "translations/lcm_viewer_draw_to_ign_model_v.h"

namespace delphyne {

// @brief Checks that an LCM viewer draw message on the input port is correctly
// translated into an ignition Model_V message.
GTEST_TEST(LCMViewerDrawToIgnModelVTranslatorSystemTest, TestTranslation) {
  const drake::lcmt_viewer_draw lcm_msg{test::BuildPreloadedDrawMsg()};

  const LcmViewerDrawToIgnModelV translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex, drake::Value<drake::lcmt_viewer_draw>(lcm_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output = translator.AllocateOutput();
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg = output->get_data(kPortIndex)->get_value<ignition::msgs::Model_V>();

  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg));
}

}  // namespace delphyne
