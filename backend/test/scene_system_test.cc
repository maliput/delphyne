// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"

#include <chrono>
#include <thread>

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "backend/test/helpers.h"

namespace delphyne {
namespace backend {

// Checks that a scene system is created, based on the Model_V on the system's
// input port.
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

  // All of the models in the original Model_V message must be present in the
  // scene message, and be equal to their counterparts.

  EXPECT_EQ(model_v_msg.models_size(), scene_msg.model_size());

  for (int model_v_idx = 0; model_v_idx < model_v_msg.models_size();
       ++model_v_idx) {
    int scene_model_idx;
    for (scene_model_idx = 0; scene_model_idx < scene_msg.model_size();
         ++scene_model_idx) {
      if (scene_msg.model(scene_model_idx).id() ==
          model_v_msg.models(model_v_idx).id()) {
        break;
      }
    }
    EXPECT_LT(scene_model_idx, scene_msg.model_size());
    EXPECT_TRUE(test::CheckProtobufMsgEquality(
        model_v_msg.models(model_v_idx), scene_msg.model(scene_model_idx)));
  }
}

}  // namespace backend
}  // namespace delphyne
