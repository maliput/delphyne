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
#include "visualization/box_car_vis.h"

#include <Eigen/Dense>
#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/systems/rendering/pose_bundle.h>
#include <drake/systems/rendering/pose_vector.h>
#include <gtest/gtest.h>

#include "test_utilities/eigen_matrix_compare.h"

using std::vector;

namespace delphyne {

using drake::systems::rendering::PoseBundle;
using drake::systems::rendering::PoseVector;

namespace {

GTEST_TEST(BoxCarVisTest, BasicTest) {
  const int kModelInstanceId = 1600;
  const std::string kName = "Alice";

  // Instantiates the device under test (DUT).
  BoxCarVis<double> dut(kModelInstanceId, kName);

  const vector<drake::lcmt_viewer_link_data>& vis_elements = dut.GetVisElements();
  EXPECT_EQ(vis_elements.size(), 1u);

  const drake::lcmt_viewer_link_data& link_data = vis_elements.at(0);
  EXPECT_EQ(link_data.name, kName);
  EXPECT_EQ(link_data.robot_num, kModelInstanceId);
  EXPECT_EQ(link_data.num_geom, 1);

  const drake::lcmt_viewer_geometry_data& geom_data = link_data.geom.at(0);
  const int kExpectedGeomType = drake::lcmt_viewer_geometry_data::BOX;
  EXPECT_EQ(geom_data.type, kExpectedGeomType);

  PoseVector<double> root_pose;
  root_pose.set_translation({1, 2, 3});
  root_pose.set_rotation({0, 0, 0, -1});
  const PoseBundle<double> vis_poses = dut.CalcPoses(root_pose.get_transform().GetAsIsometry3());
  EXPECT_EQ(vis_poses.get_num_poses(), 1);

  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  {
    expected_pose.translation().x() = 1;
    expected_pose.translation().y() = 2;
    expected_pose.translation().z() = 3;
    expected_pose.rotate(Eigen::Quaterniond({0, 0, 0, -1}));
  }
  // The following tolerance was empirically determined.
  EXPECT_TRUE(CompareMatrices(vis_poses.get_transform(0).GetAsIsometry3().matrix(), expected_pose.matrix(),
                              1e-15 /* tolerance */));

  EXPECT_EQ(vis_poses.get_model_instance_id(0), kModelInstanceId);
  EXPECT_EQ(vis_poses.get_name(0), kName);
}

}  // namespace
}  // namespace delphyne
