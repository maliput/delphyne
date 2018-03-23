// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"

#include <chrono>
#include <thread>

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

GTEST_TEST(SceneSystemTest, CalcSceneTest) {
  const ignition::msgs::Model_V model_v_msg{test::BuildPreloadedModelVMsg()};

  SceneSystem scene_system;
  std::unique_ptr<drake::systems::Context<double>> context =
      scene_system.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex,
                        drake::systems::AbstractValue::Make(model_v_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      scene_system.AllocateOutput(*context);
  scene_system.CalcOutput(*context, output.get());

  const auto& scene_msg =
      output->get_data(kPortIndex)->GetValue<ignition::msgs::Scene>();

  for (int i = 0; i < model_v_msg.models_size(); ++i) {
    int j;
    for (j = 0; j < scene_msg.model_size(); ++j) {
      if (scene_msg.model(j).id() == model_v_msg.models(i).id()) {
        break;
      }
    }
    EXPECT_LT(j, scene_msg.model_size());
    EXPECT_TRUE(test::CheckProtobufMsgEquality(model_v_msg.models(i),
                                               scene_msg.model(j)));
  }
}

}  // namespace backend
}  // namespace delphyne
