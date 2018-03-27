// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"

#include <algorithm>
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

  for (const ::ignition::msgs::Model& vector_model : model_v_msg.models()) {
    const google::protobuf::internal::RepeatedPtrIterator<
        const ignition::msgs::Model>& matching_scene_model =
        std::find_if(
            scene_msg.model().begin(), scene_msg.model().end(),
            [vector_model](const ::ignition::msgs::Model& scene_model) {
              return scene_model.id() == vector_model.id();
            });

    ASSERT_FALSE(matching_scene_model == scene_msg.model().end());

    EXPECT_TRUE(
        test::CheckProtobufMsgEquality(vector_model, *matching_scene_model));
  }
}

}  // namespace backend
}  // namespace delphyne
