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
#include "systems/pure_pursuit_controller.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "delphyne/roads/road_builder.h"
#include "test_utilities/scalar_conversion.h"

namespace delphyne {
namespace {

using drake::AutoDiffXd;
using drake::systems::rendering::PoseVector;

constexpr double kXPosition{10.};
constexpr double kLaneWidth{4.};

class PurePursuitControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a straight road with two lanes.
    road_network_ = roads::CreateDragway("One-Lane Dragway", 1 /* num_lanes */, 100 /* length */,
                                         kLaneWidth /* lane_width */, 0. /* shoulder_width */, 5. /* maximum_height */);
    road_ = road_network_->road_geometry();
    // Store the LaneDirection.
    lane_direction_.reset(new LaneDirection(road_->junction(0)->segment(0)->lane(0), true));

    // Initialize PurePursuitController with the dragway.
    dut_.reset(new PurePursuitController<double>());
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
  }

  // Create poses for one ego car and two traffic cars.
  void SetDefaultInputs(const double y_position, const double yaw) {
    // Set the LaneId.
    context_->FixInputPort(dut_->lane_input().get_index(), drake::Value<LaneDirection>(*lane_direction_));

    // Set the ego car's pose.
    PoseVector<double> ego_pose(
        Eigen::Quaternion<double>(std::cos(yaw * 0.5) /* w */, 0. /* x */, 0. /* y */, std::sin(yaw * 0.5) /* z */),
        Eigen::Translation3d(kXPosition /* x */, y_position /* y */, 0. /* z */));
    context_->FixInputPort(dut_->ego_pose_input().get_index(),
                           drake::Value<drake::systems::BasicVector<double>>(ego_pose));
  }

  std::unique_ptr<PurePursuitController<double>> dut_;  //< The device under
                                                        //  test.
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::SystemOutput<double>> output_;
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_;
  const maliput::api::RoadGeometry* road_{};
  std::unique_ptr<LaneDirection> lane_direction_;
};

TEST_F(PurePursuitControllerTest, Topology) {
  ASSERT_EQ(2, dut_->num_input_ports());
  const auto& lane_input_port = dut_->get_input_port(dut_->lane_input().get_index());
  EXPECT_EQ(drake::systems::kAbstractValued, lane_input_port.get_data_type());
  const auto& ego_input_port = dut_->get_input_port(dut_->ego_pose_input().get_index());
  EXPECT_EQ(drake::systems::kVectorValued, ego_input_port.get_data_type());
  EXPECT_EQ(7 /* PoseVector input */, ego_input_port.size());

  ASSERT_EQ(1, dut_->num_output_ports());
  const auto& command_output_port = dut_->get_output_port(dut_->steering_command_output().get_index());
  EXPECT_EQ(drake::systems::kVectorValued, command_output_port.get_data_type());
  EXPECT_EQ(1 /* steering angle output */, command_output_port.size());
}

TEST_F(PurePursuitControllerTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& other_dut) {
    auto other_context = other_dut.CreateDefaultContext();
    auto other_output = other_dut.AllocateOutput();
    auto other_derivatives = other_dut.AllocateTimeDerivatives();

    other_context->FixInputPort(dut_->lane_input().get_index(), drake::Value<LaneDirection>(*lane_direction_));
    other_context->FixInputPort(dut_->ego_pose_input().get_index(),
                                drake::Value<drake::systems::BasicVector<AutoDiffXd>>(PoseVector<AutoDiffXd>()));

    other_dut.CalcOutput(*other_context, other_output.get());
  }));
}

TEST_F(PurePursuitControllerTest, Output) {
  // Define a pointer to where the DrivingCommand results end up.
  const auto result = output_->get_vector_data(dut_->steering_command_output().get_index());
  ASSERT_NE(nullptr, result);

  // Set the offset to be to one the centerline with zero orientation.
  SetDefaultInputs(0., 0.);
  dut_->CalcOutput(*context_, output_.get());

  // Expect steering to be zero.
  EXPECT_EQ(0., (*result)[0]);

  // Set the offset to be to the right of the centerline with zero orientation.
  SetDefaultInputs(-1., 0.);
  dut_->CalcOutput(*context_, output_.get());

  // Expect the car to steer toward the left (positive steering angle).
  EXPECT_LT(0., (*result)[0]);

  // Set the offset to be to the left of the centerline, oriented at 90 degrees
  // with respect to the track.
  SetDefaultInputs(1., -M_PI_2);
  dut_->CalcOutput(*context_, output_.get());

  // Expect the car to steer toward the left (positive steering angle).
  EXPECT_LT(0., (*result)[0]);
}

}  // namespace
}  // namespace delphyne
