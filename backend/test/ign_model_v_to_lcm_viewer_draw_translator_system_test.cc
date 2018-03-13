// Copyright 2018 Toyota Research Institute

#include "backend/ign_model_v_to_lcm_viewer_draw_translator_system.h"

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

// @brief Checks that an ignition Model V on the input port is correctly
//        translated into an LCM viewer draw message.
GTEST_TEST(IgnModelVToLCMViewerDrawTranslatorSystemTest, TestTranslation) {
  const ignition::msgs::Model_V ign_msg{test::BuildPreloadedModelVMsg()};

  IgnModelVToLcmViewerDrawTranslatorSystem translator;
  auto context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(ign_msg));

  auto output = translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto& lcm_msg =
      output->get_data(kPortIndex)->GetValue<drake::lcmt_viewer_draw>();

  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg));
}

}  // namespace backend
}  // namespace delphyne
