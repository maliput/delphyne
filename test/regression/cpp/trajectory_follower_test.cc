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
#include "systems/trajectory_follower.h"

#include <vector>

#include <drake/common/extract_double.h>
#include <drake/systems/analysis/simulator.h>
#include <gtest/gtest.h>

#include "test_utilities/scalar_conversion.h"

namespace delphyne {
namespace {

static constexpr double kTol = 1e-12;
static constexpr double kSpeed = 10.;

using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;
using Eigen::Quaternion;
using Eigen::Vector3d;

GTEST_TEST(TrajectoryFollowerTest, Topology) {
  const std::vector<double> times{0., 1.};
  std::vector<Quaternion<double>> rotations{Quaternion<double>::Identity(), Quaternion<double>::Identity()};
  std::vector<Vector3d> translations{Vector3d::Zero(), Vector3d::Zero()};
  const TrajectoryFollower<double> follower(Trajectory::Make(times, rotations, translations));

  ASSERT_EQ(0, follower.num_input_ports());
  ASSERT_EQ(3, follower.num_output_ports());

  const auto& state_output = follower.state_output();
  EXPECT_EQ(drake::systems::kVectorValued, state_output.get_data_type());
  EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, state_output.size());

  const auto& pose_output = follower.pose_output();
  EXPECT_EQ(drake::systems::kVectorValued, pose_output.get_data_type());
  EXPECT_EQ(PoseVector<double>::kSize, pose_output.size());

  const auto& velocity_output = follower.velocity_output();
  EXPECT_EQ(drake::systems::kVectorValued, velocity_output.get_data_type());
  EXPECT_EQ(FrameVelocity<double>::kSize, velocity_output.size());

  ASSERT_FALSE(follower.HasAnyDirectFeedthrough());
}

GTEST_TEST(TrajectoryFollowerTest, Outputs) {
  using std::cos;
  using std::sin;

  struct TestCase {
    double heading;
    double distance;
  };

  const std::vector<TestCase> test_cases{
      {0., 1.},             // BR
      {M_PI_2, 1.},         // BR
      {-M_PI_2, 1.},        // BR
      {0.125 * M_PI, 10.},  // BR
      {0.125 * M_PI, 1.},   // BR
      {0.125 * M_PI, 1.},
  };

  for (const auto& it : test_cases) {
    const Quaternion<double> start_rotation{drake::math::RollPitchYaw<double>(0., 0., it.heading).ToQuaternion()};
    const Vector3d start_translation({5., 10., 0.});
    const Quaternion<double> end_rotation = start_rotation;
    const Vector3d end_translation{start_translation + drake::math::RotationMatrix<double>(start_rotation).matrix() *
                                                           Vector3d{it.distance, 0., 0.}};

    const std::vector<Quaternion<double>> rotations{start_rotation, end_rotation};
    const std::vector<Vector3d> translations{start_translation, end_translation};
    const std::vector<double> times{0., it.distance / kSpeed};
    const Trajectory trajectory = Trajectory::Make(times, rotations, translations);

    const TrajectoryFollower<double> follower(trajectory);

    // Check that the outputs are correct over the entirety of the trajectory.
    drake::systems::Simulator<double> simulator(follower);
    drake::systems::Context<double>& context = simulator.get_mutable_context();
    std::unique_ptr<drake::systems::SystemOutput<double>> outputs = follower.AllocateOutput();
    simulator.Initialize();

    const double end_time = it.distance / kSpeed;
    for (double time = 0.; time <= end_time; time += 0.1) {
      simulator.AdvanceTo(time);

      const double scalar = std::min(std::max(0.0, (time * kSpeed) / it.distance), 1.);
      const double position = scalar * it.distance;
      Eigen::Translation<double, 3> expected_translation(
          start_translation + Vector3d{cos(it.heading) * position, sin(it.heading) * position, 0.});
      follower.CalcOutput(context, outputs.get());

      // Tests the state output.
      const auto state =
          dynamic_cast<const SimpleCarState<double>*>(outputs->get_vector_data(follower.state_output().get_index()));
      EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, state->size());

      EXPECT_NEAR(expected_translation.x(), state->x(), kTol);
      EXPECT_NEAR(expected_translation.y(), state->y(), kTol);
      EXPECT_NEAR(it.heading, state->heading(), kTol);
      EXPECT_DOUBLE_EQ(kSpeed, state->velocity());

      // Tests the PoseVector output.
      const auto pose =
          dynamic_cast<const PoseVector<double>*>(outputs->get_vector_data(follower.pose_output().get_index()));
      ASSERT_NE(nullptr, pose);
      EXPECT_EQ(PoseVector<double>::kSize, pose->size());

      EXPECT_NEAR(expected_translation.x(), pose->get_translation().translation().x(), kTol);
      EXPECT_NEAR(expected_translation.y(), pose->get_translation().translation().y(), kTol);
      EXPECT_NEAR(cos(it.heading / 2), pose->get_rotation().w(), kTol);
      EXPECT_NEAR(sin(it.heading / 2), pose->get_rotation().z(), kTol);

      // Tests the FrameVelocity output.
      const auto velocity =
          dynamic_cast<const FrameVelocity<double>*>(outputs->get_vector_data(follower.velocity_output().get_index()));

      ASSERT_NE(nullptr, velocity);
      EXPECT_EQ(FrameVelocity<double>::kSize, velocity->size());

      EXPECT_NEAR(kSpeed * cos(it.heading), velocity->get_velocity().translational().x(), kTol);
      EXPECT_NEAR(kSpeed * sin(it.heading), velocity->get_velocity().translational().y(), kTol);
    }
  }
}

GTEST_TEST(TrajectoryFollowerTest, ToAutoDiff) {
  const std::vector<double> times{0., 1.};
  std::vector<Quaternion<double>> rotations{Quaternion<double>::Identity(), Quaternion<double>::Identity()};
  std::vector<Vector3d> translations{Vector3d::Zero(), Vector3d::Zero()};
  const TrajectoryFollower<double> follower(Trajectory::Make(times, rotations, translations));

  EXPECT_TRUE(is_autodiffxd_convertible(follower, [&](const auto& autodiff_dut) {
    auto context = autodiff_dut.CreateDefaultContext();
    auto output = autodiff_dut.AllocateOutput();

    // Check that the public methods can be called without exceptions.
    autodiff_dut.CalcOutput(*context, output.get());
  }));

  EXPECT_TRUE(is_symbolic_convertible(follower));
}

}  // namespace
}  // namespace delphyne
