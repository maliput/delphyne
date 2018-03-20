// Copyright 2018 Toyota Research Institute

#include "backend/translation_systems/lcm_viewer_draw_to_ign_model_v.h"

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

// @brief Checks that an LCM viewer draw message on the input port is correctly
//        translated into an ignition Model_V message.
GTEST_TEST(LCMViewerDrawToIgnModelVTranslatorSystemTest, TestTranslation) {
  const drake::lcmt_viewer_draw lcm_msg{test::BuildPreloadedDrawMsg()};

  translation_systems::LcmViewerDrawToIgnModelV translator;
  std::unique_ptr<drake::systems::Context<double>> context =
      translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(lcm_msg));

  auto output = translator.AllocateOutput(*context);
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg =
      output->get_data(kPortIndex)->GetValue<ignition::msgs::Model_V>();

  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg));
}

}  // namespace backend
}  // namespace delphyne
