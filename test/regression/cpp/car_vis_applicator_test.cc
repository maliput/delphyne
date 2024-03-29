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
#include "visualization/car_vis_applicator.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <drake/common/value.h>
#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <drake/systems/rendering/pose_vector.h>
#include <gtest/gtest.h>

#include "test_utilities/eigen_matrix_compare.h"
#include "visualization/box_car_vis.h"
#include "visualization/car_vis.h"

using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;

namespace delphyne {

using drake::AbstractValue;
using drake::Value;
using drake::math::RigidTransform;
using drake::systems::rendering::PoseBundle;
using drake::systems::rendering::PoseVector;

namespace {

class CarVisApplicatorTest : public ::testing::Test {
 protected:
  void SetUp() override { dut_.reset(new CarVisApplicator<double>()); }

  void CreateOutputAndContext() {
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
  }

  void SetInput(const PoseBundle<double>& pose_bundle) {
    ASSERT_NE(dut_, nullptr);
    ASSERT_NE(context_, nullptr);
    const int kPoseIndex = dut_->get_car_poses_input_port().get_index();
    context_->FixInputPort(kPoseIndex, drake::Value<PoseBundle<double>>(pose_bundle));
  }

  const PoseBundle<double>& GetOutput() const {
    const int kOutputIndex = dut_->get_visual_geometry_poses_output_port().get_index();
    return output_->get_data(kOutputIndex)->get_value<PoseBundle<double>>();
  }

  const int kIdZero{0};
  std::unique_ptr<CarVisApplicator<double>> dut_;  //< The device under test.
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::SystemOutput<double>> output_;
};

TEST_F(CarVisApplicatorTest, Topology) {
  ASSERT_EQ(dut_->num_input_ports(), 1);
  const auto& pose_port = dut_->get_car_poses_input_port();
  EXPECT_EQ(pose_port.get_data_type(), drake::systems::kAbstractValued);

  ASSERT_EQ(dut_->num_output_ports(), 1);
  const auto& lane_output_port = dut_->get_visual_geometry_poses_output_port();
  EXPECT_EQ(lane_output_port.get_data_type(), drake::systems::kAbstractValued);

  EXPECT_TRUE(dut_->HasAnyDirectFeedthrough());
}

TEST_F(CarVisApplicatorTest, Configuration) {
  EXPECT_NO_THROW(dut_->AddCarVis(make_unique<BoxCarVis<double>>(kIdZero, "Alice")));
  EXPECT_THROW(dut_->AddCarVis(make_unique<BoxCarVis<double>>(kIdZero, "Bob")), std::runtime_error);

  const int kFooId{kIdZero + 5};  // Out of order with respect to kIdZero.
  EXPECT_NO_THROW(dut_->AddCarVis(make_unique<BoxCarVis<double>>(kFooId, "Foo")));

  // Tests CarVisApplicator::get_load_robot_message().
  const int kExpectedGeomType = drake::lcmt_viewer_geometry_data::BOX;
  const drake::lcmt_viewer_load_robot& load_message = dut_->get_load_robot_message();
  EXPECT_EQ(load_message.num_links, 2);
  const int alice_index = load_message.link.at(0).name == "Alice" ? 0 : 1;
  EXPECT_EQ(load_message.link.at(alice_index).name, "Alice");
  EXPECT_EQ(load_message.link.at(alice_index).robot_num, kIdZero);
  EXPECT_EQ(load_message.link.at(alice_index).num_geom, 1);
  EXPECT_EQ(load_message.link.at(alice_index).geom.at(0).type, kExpectedGeomType);
  const int foo_index = load_message.link.at(0).name == "Foo" ? 0 : 1;
  EXPECT_EQ(load_message.link.at(foo_index).name, "Foo");
  EXPECT_EQ(load_message.link.at(foo_index).robot_num, kFooId);
  EXPECT_EQ(load_message.link.at(foo_index).num_geom, 1);
  EXPECT_EQ(load_message.link.at(foo_index).geom.at(0).type, kExpectedGeomType);

  EXPECT_EQ(dut_->num_cars(), 2);
  EXPECT_EQ(dut_->num_vis_poses(), 2);
}

TEST_F(CarVisApplicatorTest, InputOutput) {
  EXPECT_NO_THROW(dut_->AddCarVis(make_unique<BoxCarVis<double>>(kIdZero, "Alice")));
  CreateOutputAndContext();
  Eigen::Isometry3d test_pose = Eigen::Isometry3d::Identity();
  {
    const drake::math::RollPitchYaw<double> rpy(0.1, 0.5, 1.57);
    const Eigen::Vector3d xyz(4, -5, 6);
    test_pose.matrix() << rpy.ToMatrix3ViaRotationMatrix(), xyz, 0, 0, 0, 1;
  }

  PoseBundle<double> input_poses(1 /* num poses */);
  input_poses.set_transform(0, RigidTransform<double>(test_pose));
  input_poses.set_name(0, "Alice");
  input_poses.set_model_instance_id(0, kIdZero);
  SetInput(input_poses);

  dut_->CalcOutput(*context_, output_.get());

  const PoseBundle<double>& pose_bundle = GetOutput();
  EXPECT_EQ(pose_bundle.get_num_poses(), 1);
  EXPECT_TRUE(CompareMatrices(pose_bundle.get_transform(0).GetAsIsometry3().matrix(), test_pose.matrix(), 1e-15));
  EXPECT_EQ(pose_bundle.get_name(0), "Alice");
  EXPECT_EQ(pose_bundle.get_model_instance_id(0), kIdZero);
}

// Verifies that a bad PoseBundle input that contains an undefined model
// instance ID or an undefined name results in an exception being thrown.
TEST_F(CarVisApplicatorTest, BadInput) {
  EXPECT_NO_THROW(dut_->AddCarVis(make_unique<BoxCarVis<double>>(kIdZero, "Alice")));
  CreateOutputAndContext();
  Eigen::Isometry3d test_pose = Eigen::Isometry3d::Identity();
  {
    const drake::math::RollPitchYaw<double> rpy(0.1, 0.5, 1.57);
    const Eigen::Vector3d xyz(4, -5, 6);
    test_pose.matrix() << rpy.ToMatrix3ViaRotationMatrix(), xyz, 0, 0, 0, 1;
  }

  // Use a model instance ID that doesn't match the visualizer's ID.
  const int kBadId{kIdZero + 1};
  PoseBundle<double> input_poses(1 /* num poses */);
  input_poses.set_transform(0, RigidTransform<double>(test_pose));
  input_poses.set_name(0, "Alice");
  input_poses.set_model_instance_id(0, kBadId);
  SetInput(input_poses);
  EXPECT_THROW(dut_->CalcOutput(*context_, output_.get()), std::runtime_error);

  // Fixes the ID.
  input_poses.set_model_instance_id(0, kIdZero);
  SetInput(input_poses);
  EXPECT_NO_THROW(dut_->CalcOutput(*context_, output_.get()));

  // Breaks the name.
  input_poses.set_name(0, "Bob");
  SetInput(input_poses);
  EXPECT_THROW(dut_->CalcOutput(*context_, output_.get()), std::runtime_error);
}

}  // namespace
}  // namespace delphyne
