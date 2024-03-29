// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "systems/right_of_way_system.h"

#include <memory>
#include <vector>

#include <drake/systems/rendering/pose_vector.h>
#include <gtest/gtest.h>
#include <maliput/test_utilities/mock.h>

#include "delphyne/roads/road_builder.h"
#include "systems/lane_direction.h"
#include "test_utilities/scalar_conversion.h"

namespace delphyne {
namespace {

using drake::systems::rendering::PoseVector;

class RightOfWaySystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Initialize PurePursuitController with the dragway.
    dut_.reset(new RightOfWaySystem<double>(rn_.get()));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
  }

  void SetDefaultInputs(double velocity) {
    // Set the lane input.
    context_->FixInputPort(dut_->lane_state_input().get_index(), drake::Value<LaneDirection>(*lane_direction_));

    // Set the velocity input.
    context_->FixInputPort(dut_->velocity_input().get_index(),
                           drake::Value<drake::systems::BasicVector<double>>(std::initializer_list<double>{velocity}));

    // Set the pose input.
    PoseVector<double> pose_input(Eigen::Quaternion<double>(0. /* w */, 0. /* x */, 0. /* y */, 0. /* z */),
                                  Eigen::Translation3d(0. /* x */, 0. /* y */, 0. /* z */));
    context_->FixInputPort(dut_->pose_input().get_index(),
                           drake::Value<drake::systems::BasicVector<double>>(pose_input));
  }

  std::unique_ptr<RightOfWaySystem<double>> dut_;  //< The device under
                                                   //  test.
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::SystemOutput<double>> output_;
  std::unique_ptr<maliput::api::RoadNetwork> rn_ = maliput::api::test::CreateRoadNetwork();
  std::unique_ptr<maliput::api::Lane> lane_ =
      std::make_unique<maliput::api::test::MockLane>(maliput::api::LaneId{"mock"});
  std::unique_ptr<LaneDirection> lane_direction_ = std::make_unique<LaneDirection>(lane_.get());
};

TEST_F(RightOfWaySystemTest, Topology) {
  ASSERT_EQ(3, dut_->num_input_ports());
  const auto& lane_state_input_port = dut_->get_input_port(dut_->lane_state_input().get_index());
  EXPECT_EQ(drake::systems::kAbstractValued, lane_state_input_port.get_data_type());
  const auto& pose_input_port = dut_->get_input_port(dut_->pose_input().get_index());
  EXPECT_EQ(drake::systems::kVectorValued, pose_input_port.get_data_type());
  EXPECT_EQ(7 /* Pose vector input */, pose_input_port.size());
  const auto& velocity_input_port = dut_->get_input_port(dut_->velocity_input().get_index());
  EXPECT_EQ(drake::systems::kVectorValued, velocity_input_port.get_data_type());
  EXPECT_EQ(1 /* Velocity input */, velocity_input_port.size());

  ASSERT_EQ(1, dut_->num_output_ports());
  const auto& velocity_output_port = dut_->get_output_port(dut_->velocity_output().get_index());
  EXPECT_EQ(drake::systems::kVectorValued, velocity_output_port.get_data_type());
  EXPECT_EQ(1 /* Velocity output */, velocity_output_port.size());
}

// The RoadNetwork doesn't have Right of Way rules so the velocity will be passed through.
TEST_F(RightOfWaySystemTest, OutputNoRules) {
  // Define a pointer to where the Velocity results end up.
  const auto result = output_->get_vector_data(dut_->velocity_output().get_index());
  ASSERT_NE(nullptr, result);

  // Set the offset to be to one the centerline with zero orientation.
  const double kVelocity{11.};
  SetDefaultInputs(kVelocity);
  dut_->CalcOutput(*context_, output_.get());

  // Expect velocity to be kVelocity.
  EXPECT_EQ(kVelocity, (*result)[0]);
}

// TODO(francocipollone): Adds a tests where the right of way rule's states causes the output to be zero.

}  // namespace
}  // namespace delphyne
