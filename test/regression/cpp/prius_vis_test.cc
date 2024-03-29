// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "visualization/prius_vis.h"

#include <stdexcept>
#include <string>
#include <vector>

#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/math/rotation_matrix.h>
#include <drake/systems/rendering/frame_velocity.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <gtest/gtest.h>

#include "test_utilities/eigen_matrix_compare.h"

namespace delphyne {

using drake::Isometry3;
using drake::lcmt_viewer_link_data;
using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseBundle;

namespace {

// Searches the provided PoseBundle for the pose of "chassis_floor" and returns
// a reference to it if found. It throws a std::runtime_error exception if it
// fails to find the pose.
const Isometry3<double> GetChassisFloorPose(const PoseBundle<double>& poses) {
  for (int i = 0; i < poses.get_num_poses(); ++i) {
    if (poses.get_name(i) == "chassis_floor") {
      return poses.get_transform(i).GetAsIsometry3();
    }
  }
  throw std::runtime_error("Failed to find pose of chassis_floor.");
}

GTEST_TEST(PriusVisTest, BasicTest) {
  const int kModelInstanceId = 1600;
  const std::string kName = "MyPrius";
  const int kNumBodies = 16;

  // Instantiates the device under test (DUT).
  PriusVis<double> dut(kModelInstanceId, kName);

  const std::vector<lcmt_viewer_link_data>& vis_elements = dut.GetVisElements();
  EXPECT_EQ(static_cast<int>(vis_elements.size()), kNumBodies);

  // Ensures the visualization elements do not include the world.
  for (const auto& link : vis_elements) {
    EXPECT_NE(link.name, "world");
  }

  const lcmt_viewer_link_data& first_link_data = vis_elements.at(0);
  EXPECT_EQ(first_link_data.robot_num, kModelInstanceId);
  EXPECT_EQ(first_link_data.num_geom, 1);

  // Evaluates the poses when X_WM_W is identity. This means the visualization
  // model's root is at the origin.
  const Eigen::Isometry3d X_WM_W_origin = Eigen::Isometry3d::Identity();
  PoseBundle<double> origin_vis_poses = dut.CalcPoses(X_WM_W_origin);
  EXPECT_EQ(origin_vis_poses.get_num_poses(), kNumBodies);

  // Defines visualization model translation offsets to evaluate. All poses in
  // the resulting visualization PoseBundle should be offset by these amounts
  // relative to `origin_vis_poses`.
  const std::vector<std::vector<double>> translation_offsets = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1},
                                                                {1, 1, 0}, {0, 1, 1}, {1, 1, 1}};

  for (const auto& translation : translation_offsets) {
    const int xOffset = translation.at(0);
    const int yOffset = translation.at(1);
    const int zOffset = translation.at(2);

    const Eigen::Isometry3d X_WM_W_offset(Eigen::Translation3d(xOffset, yOffset, zOffset));
    PoseBundle<double> offset_vis_poses = dut.CalcPoses(X_WM_W_offset);
    EXPECT_EQ(offset_vis_poses.get_num_poses(), kNumBodies);
    ASSERT_EQ(origin_vis_poses.get_num_poses(), offset_vis_poses.get_num_poses());

    // Verifies that all of the poses in `offset_vis_poses` are offset as
    // expected relative to `origin_vis_poses`, but are otherwise identical.
    for (int i = 0; i < origin_vis_poses.get_num_poses(); ++i) {
      EXPECT_EQ(origin_vis_poses.get_name(i), offset_vis_poses.get_name(i));
      EXPECT_EQ(origin_vis_poses.get_model_instance_id(i), offset_vis_poses.get_model_instance_id(i));

      const Isometry3<double>& offset_pose = offset_vis_poses.get_transform(i).GetAsIsometry3();
      const Isometry3<double> expected_pose =
          Eigen::Translation3d(xOffset, yOffset, zOffset) * origin_vis_poses.get_transform(i).GetAsIsometry3();

      EXPECT_TRUE(CompareMatrices(offset_pose.matrix(), expected_pose.matrix(), 1e-15, MatrixCompareType::absolute));

      const FrameVelocity<double>& frame_velocity_origin = origin_vis_poses.get_velocity(i);
      const FrameVelocity<double>& frame_velocity_higher = offset_vis_poses.get_velocity(i);
      EXPECT_EQ(frame_velocity_origin.get_value(), frame_velocity_higher.get_value());
    }
  }

  // Defines visualization model rotation offsets to evaluate, in roll-pitch-yaw
  // order. All poses in the resulting visualization PoseBundle should be offset
  // by these amounts relative to `origin_vis_poses`.
  const std::vector<Eigen::Vector3d> rotation_offsets = {
      Eigen::Vector3d(M_PI_2, 0, 0),      Eigen::Vector3d(0, M_PI_2, 0),      Eigen::Vector3d(0, 0, M_PI_2),
      Eigen::Vector3d(M_PI_2, M_PI_2, 0), Eigen::Vector3d(0, M_PI_2, M_PI_2), Eigen::Vector3d(M_PI_2, 0, M_PI_2),
  };

  for (const auto& rotation : rotation_offsets) {
    const drake::math::RollPitchYaw<double> rpy(rotation);
    Eigen::Isometry3d X_WM_W_offset = rpy.ToQuaternion() * X_WM_W_origin;
    PoseBundle<double> offset_vis_poses = dut.CalcPoses(X_WM_W_offset);
    EXPECT_EQ(offset_vis_poses.get_num_poses(), kNumBodies);
    ASSERT_EQ(origin_vis_poses.get_num_poses(), offset_vis_poses.get_num_poses());

    // Verifies that all of the poses in `offset_vis_poses` are rotated as
    // expected relative to `origin_vis_poses` but are otherwise identical.
    for (int i = 0; i < origin_vis_poses.get_num_poses(); ++i) {
      EXPECT_EQ(origin_vis_poses.get_name(i), offset_vis_poses.get_name(i));
      EXPECT_EQ(origin_vis_poses.get_model_instance_id(i), offset_vis_poses.get_model_instance_id(i));

      const Isometry3<double>& offset_pose = offset_vis_poses.get_transform(i).GetAsIsometry3();
      const Isometry3<double> expected_pose =
          rpy.ToMatrix3ViaRotationMatrix() * origin_vis_poses.get_transform(i).GetAsIsometry3();
      ASSERT_TRUE(CompareMatrices(offset_pose.linear(), expected_pose.linear(), 1e-15, MatrixCompareType::absolute));

      const FrameVelocity<double>& frame_velocity_origin = origin_vis_poses.get_velocity(i);
      const FrameVelocity<double>& frame_velocity_higher = offset_vis_poses.get_velocity(i);
      EXPECT_EQ(frame_velocity_origin.get_value(), frame_velocity_higher.get_value());
    }
  }

  // Tests the visualization's pose when the model is rotated 90 degrees about
  // its +Z axis.
  const Isometry3<double> floor_pose_identity = GetChassisFloorPose(origin_vis_poses);
  EXPECT_DOUBLE_EQ(floor_pose_identity.translation().x(), 0.);

  // Tests the visualization's pose when the model is rotated 90 degrees about
  // its +Z axis. In other words, the vehicle is facing left.
  const Eigen::Isometry3d X_WM_W_90_about_z =
      drake::math::RollPitchYaw<double>(Eigen::Vector3d(0, 0, M_PI_2)).ToQuaternion() * Eigen::Isometry3d::Identity();
  EXPECT_DOUBLE_EQ(GetChassisFloorPose(dut.CalcPoses(X_WM_W_90_about_z)).translation().y(), 0.);

  // Tests the visualization's pose when the model is rotated 45 degrees about
  // its +Y axis. In other words, the vehicle is going down a steep hill.
  const Eigen::Isometry3d X_WM_W_45_about_y =
      drake::math::RollPitchYaw<double>(Eigen::Vector3d(0, M_PI_4, 0)).ToQuaternion() * Eigen::Isometry3d::Identity();
  const Isometry3<double> floor_pose_down_hill = GetChassisFloorPose(dut.CalcPoses(X_WM_W_45_about_y));
  EXPECT_DOUBLE_EQ(floor_pose_down_hill.translation().x(), floor_pose_identity.translation().z() * std::sin(M_PI_4));
  EXPECT_DOUBLE_EQ(floor_pose_down_hill.translation().z(), floor_pose_identity.translation().z() * std::cos(M_PI_4));

  // Tests the visualization's pose when the model is rotated 45 degrees about
  // its +X axis. In other words, the vehicle is leaning to its right side due
  // to the lane being severely cambered.
  const Eigen::Isometry3d X_WM_W_45_about_x =
      drake::math::RollPitchYaw<double>(Eigen::Vector3d(M_PI_4, 0, 0)).ToQuaternion() * Eigen::Isometry3d::Identity();
  const Isometry3<double> floor_pose_severe_camber = GetChassisFloorPose(dut.CalcPoses(X_WM_W_45_about_x));
  EXPECT_DOUBLE_EQ(floor_pose_severe_camber.translation().x(), 0.);
}

}  // namespace
}  // namespace delphyne
