// Copyright 2018 Toyota Research Institute

#include "backend/ign_models_assembler.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include <drake/common/eigen_types.h>
#include <drake/systems/framework/framework_common.h>
#include <drake/systems/rendering/pose_bundle.h>

#include <gtest/gtest.h>

#include "test_utilities/helpers.h"

namespace delphyne {
namespace {

using drake::AngleAxis;
using drake::Isometry3;
using drake::Translation3;
using drake::Vector3;
using drake::systems::rendering::PoseBundle;

// Checks that the ignition::msgs::Model_V assembly process works as expected.
GTEST_TEST(IgnModelsAssemblerTest, CalcAssembledModelVTest) {
  const ignition::msgs::Model_V input_models{test::BuildPreloadedModelVMsg()};

  const PoseBundle<double> input_states{test::BuildPreloadedPoseBundle()};

  IgnModelsAssembler models_assembler;
  std::unique_ptr<drake::systems::Context<double>> context = models_assembler.AllocateContext();

  context->FixInputPort(models_assembler.get_models_input_port_index(), drake::AbstractValue::Make(input_models));

  context->FixInputPort(models_assembler.get_states_input_port_index(), drake::AbstractValue::Make(input_states));

  std::unique_ptr<drake::systems::SystemOutput<double>> output = models_assembler.AllocateOutput();
  models_assembler.CalcOutput(*context, output.get());

  const double kOutputPortIndex{0};
  const auto& output_models = output->get_data(kOutputPortIndex)->get_value<ignition::msgs::Model_V>();

  EXPECT_EQ(input_models.models_size(), output_models.models_size());

  for (int i = 0; i < output_models.models_size(); ++i) {
    // Output models should match input models but on
    // model name and pose.
    EXPECT_TRUE(test::CheckProtobufMsgEquality(input_models.models(i).header(), output_models.models(i).header()));
    EXPECT_NE(input_models.models(i).name(), output_models.models(i).name());
    EXPECT_EQ(input_models.models(i).id(), output_models.models(i).id());
    EXPECT_EQ(input_models.models(i).is_static(), output_models.models(i).is_static());
    EXPECT_FALSE(test::CheckProtobufMsgEquality(input_models.models(i).pose(), output_models.models(i).pose()));
    ASSERT_EQ(input_models.models(i).joint().size(), output_models.models(i).joint().size());
    for (int j = 0; j < input_models.models(i).joint().size(); ++j) {
      EXPECT_TRUE(test::CheckProtobufMsgEquality(input_models.models(i).joint(j), output_models.models(i).joint(j)));
    }
    ASSERT_EQ(input_models.models(i).link().size(), output_models.models(i).link().size());
    for (int j = 0; j < input_models.models(i).link().size(); ++j) {
      EXPECT_TRUE(test::CheckProtobufMsgEquality(input_models.models(i).link(j), output_models.models(i).link(j)));
    }
    EXPECT_EQ(input_models.models(i).deleted(), output_models.models(i).deleted());
    ASSERT_EQ(input_models.models(i).visual().size(), output_models.models(i).visual().size());
    for (int j = 0; j < input_models.models(i).visual().size(); ++j) {
      EXPECT_TRUE(test::CheckProtobufMsgEquality(input_models.models(i).visual(j), output_models.models(i).visual(j)));
    }
    EXPECT_TRUE(test::CheckProtobufMsgEquality(input_models.models(i).scale(), output_models.models(i).scale()));
    EXPECT_EQ(input_models.models(i).self_collide(), output_models.models(i).self_collide());

    // Output model name should match that in input states.
    EXPECT_EQ(output_models.models(i).name(), input_states.get_name(i));

    // Output model pose should match that in input states.
    const ignition::msgs::Pose& output_model_pose = output_models.models(i).pose();
    const Isometry3<double>& pose = input_states.get_pose(i);
    const drake::Vector3<double>& position = pose.translation();
    EXPECT_EQ(position.x(), output_model_pose.position().x());
    EXPECT_EQ(position.y(), output_model_pose.position().y());
    EXPECT_EQ(position.z(), output_model_pose.position().z());
    const drake::Quaternion<double> orientation(pose.linear());
    EXPECT_EQ(orientation.x(), output_model_pose.orientation().x());
    EXPECT_EQ(orientation.y(), output_model_pose.orientation().y());
    EXPECT_EQ(orientation.z(), output_model_pose.orientation().z());
    EXPECT_EQ(orientation.w(), output_model_pose.orientation().w());
  }
}

}  // namespace
}  // namespace delphyne
