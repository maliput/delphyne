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
  FramePoseAggregator<double> frame_pose_aggregator;

  const drake::geometry::FrameId frame0 = drake::geometry::FrameId::get_new_id();
  const drake::systems::InputPort<double>& input_pose_port0 = frame_pose_aggregator.DeclareInput(frame0);
  const drake::geometry::FrameId frame1 = drake::geometry::FrameId::get_new_id();
  const drake::systems::InputPort<double>& input_pose_port1 = frame_pose_aggregator.DeclareInput(frame1);

  std::unique_ptr<drake::systems::Context<double>> context = frame_pose_aggregator.AllocateContext();

  auto input_pose0 = std::make_unique<drake::systems::rendering::PoseVector<double>>(
      drake::Quaternion<double>(0.5, 0.5, 0., 0.), drake::Translation3<double>(1., 10., 100.));
  context->FixInputPort(input_pose_port0.get_index(), input_pose0->Clone());

  auto input_pose1 = std::make_unique<drake::systems::rendering::PoseVector<double>>(
      drake::Quaternion<double>(0.5, 0., 0.5, 0.), drake::Translation3<double>(-1., -10., -100.));
  context->FixInputPort(input_pose_port1.get_index(), input_pose1->Clone());

  std::unique_ptr<drake::systems::SystemOutput<double>> output = frame_pose_aggregator.AllocateOutput();
  frame_pose_aggregator.CalcOutput(*context, output.get());

  const drake::AbstractValue* output_value = output->get_data(0);
  const auto& output_frame_pose_vector = output_value->get_value<drake::geometry::FramePoseVector<double>>();

  EXPECT_EQ(output_frame_pose_vector.size(), 2);
  {
    EXPECT_TRUE(output_frame_pose_vector.has_id(frame0));
    const auto isometry = input_pose0->get_isometry();
    const auto orthogonal_rotation_matrix =
        drake::math::RotationMatrix<double>::ProjectToRotationMatrix(isometry.linear());
    drake::math::RigidTransform rigid_transform{orthogonal_rotation_matrix};
    rigid_transform.set_translation(isometry.translation());
    EXPECT_TRUE(output_frame_pose_vector.value(frame0).GetAsIsometry3().isApprox(rigid_transform.GetAsIsometry3()));
  }
  {
    EXPECT_TRUE(output_frame_pose_vector.has_id(frame1));
    const auto isometry = input_pose1->get_isometry();
    const auto orthogonal_rotation_matrix =
        drake::math::RotationMatrix<double>::ProjectToRotationMatrix(isometry.linear());
    drake::math::RigidTransform rigid_transform{orthogonal_rotation_matrix};
    rigid_transform.set_translation(isometry.translation());
    EXPECT_TRUE(output_frame_pose_vector.value(frame1).GetAsIsometry3().isApprox(rigid_transform.GetAsIsometry3()));
  }
}

}  // namespace
}  // namespace delphyne
