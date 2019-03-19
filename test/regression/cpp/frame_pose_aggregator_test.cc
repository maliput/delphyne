// Copyright 2018 Toyota Research Institute

#include "backend/frame_pose_aggregator.h"

#include <memory>

#include <drake/common/eigen_types.h>
#include <drake/common/value.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/output_port.h>
#include <drake/systems/rendering/pose_vector.h>

#include <gtest/gtest.h>

namespace delphyne {
namespace {

GTEST_TEST(FramePoseAggregatorTest, CorrectAggregation) {
  const drake::geometry::SourceId source_id =
      drake::geometry::SourceId::get_new_id();
  FramePoseAggregator<double> frame_pose_aggregator(source_id);

  const drake::geometry::FrameId frame0 =
      drake::geometry::FrameId::get_new_id();
  const drake::systems::InputPort<double>& input_pose_port0 =
      frame_pose_aggregator.DeclareInput(frame0);
  const drake::geometry::FrameId frame1 =
      drake::geometry::FrameId::get_new_id();
  const drake::systems::InputPort<double>& input_pose_port1 =
      frame_pose_aggregator.DeclareInput(frame1);

  std::unique_ptr<drake::systems::Context<double>> context =
      frame_pose_aggregator.AllocateContext();

  auto input_pose0 =
      std::make_unique<drake::systems::rendering::PoseVector<double>>(
          drake::Quaternion<double>(0.5, 0.5, 0., 0.),
          drake::Translation3<double>(1., 10., 100.));
  context->FixInputPort(input_pose_port0.get_index(), input_pose0->Clone());

  auto input_pose1 =
      std::make_unique<drake::systems::rendering::PoseVector<double>>(
          drake::Quaternion<double>(0.5, 0., 0.5, 0.),
          drake::Translation3<double>(-1., -10., -100.));
  context->FixInputPort(input_pose_port1.get_index(), input_pose1->Clone());

  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      frame_pose_aggregator.AllocateOutput();
  frame_pose_aggregator.CalcOutput(*context, output.get());

  const drake::AbstractValue* output_value = output->get_data(0);
  const auto& output_frame_pose_vector =
      output_value->get_value<drake::geometry::FramePoseVector<double>>();

  EXPECT_EQ(output_frame_pose_vector.source_id(), source_id);
  EXPECT_EQ(output_frame_pose_vector.size(), 2);
  EXPECT_TRUE(output_frame_pose_vector.has_id(frame0));
  EXPECT_TRUE(output_frame_pose_vector.value(frame0).isApprox(
      input_pose0->get_isometry()));
  EXPECT_TRUE(output_frame_pose_vector.has_id(frame1));
  EXPECT_TRUE(output_frame_pose_vector.value(frame1).isApprox(
      input_pose1->get_isometry()));
}

}  // namespace
}  // namespace delphyne
