#include "systems/unicycle_car.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "maliput_multilane_test_utilities/eigen_matrix_compare.h"

#include "gen/simple_car_state.h"

namespace delphyne {

using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;

namespace {

class UnicycleCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new UnicycleCar<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
    derivatives_ = dut_->AllocateTimeDerivatives();
    SetInputValue(0.0, 0.0);
  }

  void SetInputValue(double angular_rate, double acceleration) {
    AngularRateAccelerationCommand<double> value;
    value.set_angular_rate(angular_rate);
    value.set_acceleration(acceleration);
    context_->FixInputPort(0, drake::Value<drake::systems::BasicVector<double>>(value));
  }

  SimpleCarState<double>* continuous_state() {
    auto result = dynamic_cast<SimpleCarState<double>*>(&context_->get_mutable_continuous_state_vector());
    if (result == nullptr) {
      throw std::bad_cast();
    }
    return result;
  }

  const SimpleCarState<double>* state_output() const {
    auto state = dynamic_cast<const SimpleCarState<double>*>(output_->get_vector_data(0));
    DRAKE_DEMAND(state != nullptr);
    return state;
  }

  const PoseVector<double>* pose_output() const {
    auto pose = dynamic_cast<const PoseVector<double>*>(output_->get_vector_data(1));
    DRAKE_DEMAND(pose != nullptr);
    return pose;
  }

  const FrameVelocity<double>* velocity_output() const {
    auto velocity = dynamic_cast<const FrameVelocity<double>*>(output_->get_vector_data(2));
    DRAKE_DEMAND(velocity != nullptr);
    return velocity;
  }

  // Sets an arbitrary, nonzero state.
  void InitializeNonzeroState() {
    continuous_state()->set_x(1.0);
    continuous_state()->set_y(2.0);
    continuous_state()->set_heading(3.0);
    continuous_state()->set_velocity(4.0);
  }

  // Checks that the pose output has the correct values for the state that
  // InitializeNonzeroState sets.
  void VerifyNonzeroPose() {
    Eigen::Translation<double, 3> p_WC = pose_output()->get_translation();
    EXPECT_EQ(1.0, p_WC.translation().x());
    EXPECT_EQ(2.0, p_WC.translation().y());
    EXPECT_EQ(0.0, p_WC.translation().z());

    // A rotation about the z axis is nonzero in only the real and k parts of
    // q = w + xi + yj + zk.
    Eigen::Quaternion<double> R_WC = pose_output()->get_rotation();
    EXPECT_EQ(std::cos(3.0 / 2), R_WC.w());
    EXPECT_EQ(0.0, R_WC.x());
    EXPECT_EQ(0.0, R_WC.y());
    EXPECT_EQ(std::sin(3.0 / 2), R_WC.z());
  }

  // Checks that the velocity output has the correct values for the state that
  // InitializeNonzeroState sets.
  void VerifyNonzeroVelocity() {
    const drake::multibody::SpatialVelocity<double> v_WC = velocity_output()->get_velocity();
    EXPECT_EQ(std::cos(3.0) * 4.0, v_WC.translational()[0]);
    EXPECT_EQ(std::sin(3.0) * 4.0, v_WC.translational()[1]);
  }

  std::unique_ptr<UnicycleCar<double>> dut_;  //< The device under test.
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::SystemOutput<double>> output_;
  std::unique_ptr<drake::systems::ContinuousState<double>> derivatives_;
};

TEST_F(UnicycleCarTest, Topology) {
  ASSERT_EQ(1, dut_->num_input_ports());
  const auto& input_port = dut_->get_input_port(0);
  EXPECT_EQ(drake::systems::kVectorValued, input_port.get_data_type());
  EXPECT_EQ(AngularRateAccelerationCommand<double>::num_coordinates(), input_port.size());

  ASSERT_EQ(3, dut_->num_output_ports());
  const auto& state_output = dut_->state_output();
  EXPECT_EQ(drake::systems::kVectorValued, state_output.get_data_type());
  EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, state_output.size());

  const auto& pose_output = dut_->pose_output();
  EXPECT_EQ(drake::systems::kVectorValued, pose_output.get_data_type());
  EXPECT_EQ(PoseVector<double>::kSize, pose_output.size());

  const auto& velocity_output = dut_->velocity_output();
  EXPECT_EQ(drake::systems::kVectorValued, velocity_output.get_data_type());
  EXPECT_EQ(FrameVelocity<double>::kSize, velocity_output.size());
}

TEST_F(UnicycleCarTest, ZeroOutput) {
  auto state = state_output();
  auto pose = pose_output();

  // Starting state and output is all zeros.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, state->x());
  EXPECT_EQ(0.0, state->y());
  EXPECT_EQ(0.0, state->heading());
  EXPECT_EQ(0.0, state->velocity());

  EXPECT_TRUE(
      CompareMatrices(drake::Isometry3<double>::Identity().matrix(), pose->get_transform().GetAsIsometry3().matrix()));
}

TEST_F(UnicycleCarTest, StateAppearsInOutput) {
  // New state just propagates through.
  InitializeNonzeroState();

  auto state = state_output();

  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, state->x());
  EXPECT_EQ(2.0, state->y());
  EXPECT_EQ(3.0, state->heading());
  EXPECT_EQ(4.0, state->velocity());
  VerifyNonzeroPose();
  VerifyNonzeroVelocity();
}

TEST_F(UnicycleCarTest, InputDoesNotAffectOutput) {
  InitializeNonzeroState();

  // The input doesn't matter.
  SetInputValue(0.3, 0.5);

  auto state = state_output();

  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, state->x());
  EXPECT_EQ(2.0, state->y());
  EXPECT_EQ(3.0, state->heading());
  EXPECT_EQ(4.0, state->velocity());
  VerifyNonzeroPose();
  VerifyNonzeroVelocity();
}

TEST_F(UnicycleCarTest, Derivatives) {
  const double kTolerance = 1e-10;

  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const SimpleCarState<double>* const result =
      dynamic_cast<const SimpleCarState<double>*>(&derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting state is all zeros.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // Set acceleration a positive value.
  SetInputValue(0.0, 0.5);
  // At heading 0, we are moving along +x.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.5, result->velocity());

  // Set speed to a positive value, with zero input.
  continuous_state()->set_velocity(10.0);
  SetInputValue(0.0, 0.0);
  // At heading 0, we are moving along +x.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());

  // A non-zero angular_rate turns in the same direction.
  const double angular_rate = 5.0;
  SetInputValue(angular_rate, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(5.0, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());
  SetInputValue(-angular_rate, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(10.0, result->x(), kTolerance);
  EXPECT_EQ(0.0, result->y());
  EXPECT_NEAR(-5.0, result->heading(), kTolerance);
  EXPECT_EQ(0.0, result->velocity());

  // A heading of +90deg points us at +y.
  continuous_state()->set_velocity(10.0);
  continuous_state()->set_heading(0.5 * M_PI);
  SetInputValue(0.0, 0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR(0.0, result->x(), kTolerance);
  EXPECT_NEAR(10.0, result->y(), kTolerance);
  EXPECT_EQ(0.0, result->heading());
  EXPECT_EQ(0.0, result->velocity());
}

}  // namespace
}  // namespace delphyne
