// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include "systems/bicycle_car.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "test_utilities/eigen_matrix_compare.h"
#include "test_utilities/scalar_conversion.h"

namespace delphyne {

using drake::Vector6;

namespace {

static constexpr int kStateDimension{BicycleCarStateIndices::kNumCoordinates};
static constexpr int kSteeringInputDimension{1};
static constexpr int kForceInputDimension{1};

class BicycleCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new BicycleCar<double>());

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  BicycleCarState<double>* continuous_state() {
    auto& xc = context_->get_mutable_continuous_state_vector();
    BicycleCarState<double>* state = dynamic_cast<BicycleCarState<double>*>(&xc);
    DRAKE_DEMAND(state != nullptr);
    return state;
  }

  const BicycleCarState<double>* derivatives() const {
    const auto derivatives = dynamic_cast<const BicycleCarState<double>*>(&derivatives_->get_mutable_vector());
    DRAKE_DEMAND(derivatives != nullptr);
    return derivatives;
  }

  void SetInputs(const double steering_angle, const double force) {
    ASSERT_NE(nullptr, dut_);
    ASSERT_NE(nullptr, context_);

    drake::systems::BasicVector<double> steering_input(kSteeringInputDimension);
    steering_input[0] = steering_angle;

    drake::systems::BasicVector<double> force_input(kForceInputDimension);
    force_input[0] = force;

    const int kSteeringIndex = dut_->get_steering_input_port().get_index();
    const int kForceIndex = dut_->get_force_input_port().get_index();
    context_->FixInputPort(kSteeringIndex, drake::Value<drake::systems::BasicVector<double>>(steering_input));
    context_->FixInputPort(kForceIndex, drake::Value<drake::systems::BasicVector<double>>(force_input));
  }

  std::unique_ptr<BicycleCar<double>> dut_;  //< The device under test.
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::SystemOutput<double>> output_;
  std::unique_ptr<drake::systems::ContinuousState<double>> derivatives_;
};

TEST_F(BicycleCarTest, Topology) {
  ASSERT_EQ(2, dut_->num_input_ports()); /* steering angle, force input */

  const auto& steering_input_port = dut_->get_steering_input_port();
  EXPECT_EQ(drake::systems::kVectorValued, steering_input_port.get_data_type());
  EXPECT_EQ(kSteeringInputDimension, steering_input_port.size());

  const auto& force_input_port = dut_->get_force_input_port();
  EXPECT_EQ(drake::systems::kVectorValued, force_input_port.get_data_type());
  EXPECT_EQ(kForceInputDimension, force_input_port.size());

  ASSERT_EQ(1, dut_->num_output_ports()); /* state vector */

  const auto& state_port = dut_->get_output_port(0);
  EXPECT_EQ(drake::systems::kVectorValued, state_port.get_data_type());
  EXPECT_EQ(kStateDimension, state_port.size());
}

// clang-format off
TEST_F(BicycleCarTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_));
}

TEST_F(BicycleCarTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*dut_));
}

TEST_F(BicycleCarTest, DirectFeedthrough) {
  EXPECT_FALSE(dut_->HasAnyDirectFeedthrough());
}
// clang-format on

TEST_F(BicycleCarTest, Output) {
  const double kTolerance = 1e-10;

  // Set the steering angle and the applied force to positive values.
  SetInputs(1., 10.);

  auto output = output_->get_vector_data(0);

  // Set all the states to one.
  continuous_state()->SetFromVector(Vector6<double>::Ones());

  dut_->CalcOutput(*context_, output_.get());
  const Vector6<double> result = output->CopyToVector();

  // Expect that the output matches the states, since there is no feedthrough.
  EXPECT_TRUE(CompareMatrices(result, Vector6<double>::Ones(), kTolerance, MatrixCompareType::absolute));
}

// Tests the consistency of the derivatives when a trivial set of states is
// provided, with no steering and some positive input force.
TEST_F(BicycleCarTest, TrivialDerivatives) {
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Keep the steering angle zero and set the force to a positive value.
  SetInputs(0., kForceInput);

  // Set all the states to zero except velocity, which must be kept positive.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect all derivatives to be zero except vel_dot and sx_dot, since
  // applying a positive force input and zero steering angle translates into
  // along-track motion.
  EXPECT_EQ(0., derivatives()->Psi());
  EXPECT_EQ(0., derivatives()->Psi_dot());
  EXPECT_EQ(0., derivatives()->beta());
  EXPECT_LT(0., derivatives()->vel());
  EXPECT_EQ(kVelocityState, derivatives()->sx());
  EXPECT_EQ(0., derivatives()->sy());
}

// Tests that one equation has terms that cancel for some parameter-independent
// settings, and that the remaining equations are still consistent, including
// the kinematics equations.
TEST_F(BicycleCarTest, DerivativesPositiveBetaPositiveDelta) {
  const double kSteeringInput = 1.;  // An angle in the first quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to δ / 2 and the velocity to a positive value.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_beta(kSteeringInput / 2.);
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect the first and third terms in β_dot to cancel, and Ψ_ddot, vel_dot
  // to be strictly positive. We expect sx_dot, and sy_dot to be
  // strictly-positive as the steering angle is in the first quadrant.
  EXPECT_LT(0., derivatives()->Psi_dot());
  EXPECT_EQ(0., derivatives()->beta());
  EXPECT_LT(0., derivatives()->vel());
  EXPECT_LT(0., derivatives()->sx());
  EXPECT_LT(0., derivatives()->sy());
}

// Tests the consistency of the derivatives and kinematic relationships upon
// feeding in a negative slip angle, keeping the other parameters the same as
// the previous case.
TEST_F(BicycleCarTest, DerivativesNegativeBetaPositiveDelta) {
  const double kSteeringInput = 1.;  // An angle in the fourth quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to -δ and the velocity to a positive value.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_beta(-kSteeringInput);
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect β_dot and vel_dot to return strictly-positive values. sx_dot and
  // sy_dot, respectively, return positive and negative values, as the steering
  // angle is in the fourth quadrant.  Note that Ψ_ddot is indeterminate.
  EXPECT_LT(0., derivatives()->beta());
  EXPECT_LT(0., derivatives()->vel());
  EXPECT_LT(0., derivatives()->sx());
  EXPECT_GT(0., derivatives()->sy());
}

// Tests the consistency of the derivatives and kinematic relationships upon
// assigning a positive angular yaw rate.
TEST_F(BicycleCarTest, DerivativesPositivePsiDot) {
  const double kYawRateState = 2.;
  const double kVelocityState = 1e3;  // Unrealistic velocity that reasonably
                                      // enforces the condition that
                                      // abs(Cr * lr - Cf * lf) < 1e6 * mass.

  // Keep the steering angle and force zero.
  SetInputs(0., 0.);

  // Set Ψ_dot and the velocity to positive values.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_Psi_dot(kYawRateState);
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect both β_dot and Ψ_ddot to be strictly negative.
  EXPECT_GT(0., derivatives()->Psi_dot());
  EXPECT_GT(0., derivatives()->beta());
}

}  // namespace
}  // namespace delphyne
