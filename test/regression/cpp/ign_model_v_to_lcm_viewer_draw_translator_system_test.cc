// Copyright 2018 Toyota Research Institute

#include "drake/systems/framework/framework_common.h"

#include <gtest/gtest.h>

#include "helpers.h"
#include "translations/ign_model_v_to_lcm_viewer_draw.h"

namespace delphyne {

// @brief Checks that an ignition Model_V message on the input port is correctly
// translated into an LCM viewer draw message.
GTEST_TEST(IgnModelVToLCMViewerDrawTranslatorSystemTest, TestTranslation) {
  const ignition::msgs::Model_V ign_msg{test::BuildPreloadedModelVMsg()};

  const IgnModelVToLcmViewerDraw translator;
  std::unique_ptr<drake::systems::Context<double>> context =
      translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(ign_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto& lcm_msg =
      output->get_data(kPortIndex)->GetValue<drake::lcmt_viewer_draw>();

  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg));
}

}  // namespace delphyne
