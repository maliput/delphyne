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
#include "systems/idm_planner.h"

#include <cmath>
#include <memory>

#include <drake/common/autodiff.h>
#include <drake/common/extract_double.h>
#include <gtest/gtest.h>

#include "test_utilities/autodiff_test_utilities.h"

namespace delphyne {
namespace {

using drake::ExtractDoubleOrThrow;
using std::pow;
using std::sqrt;
using test::CheckDerivativeNegativity;
using test::CheckDerivativePositivity;
using test::CheckDerivatives;
using test::SetDerivatives;

template <typename T>
class IdmPlannerTest : public ::testing::Test {
 protected:
  // Returns ∂/∂v(result) for the special case where accel_interaction (see
  // "Fast-closing-speed behavior" in the class documentation) is zero and v =
  // v_ref, where `v` is ego_velocity and `result` is the longitudinal
  // acceleration returned by IDM.
  double get_free_dadv() const { return -delta() * a() / v_ref(); }

  // Returns ∂/∂v(result) for the special case where v = d_dot = 0 and d = 1,
  // where `v` is ego_velocity, `d` is target_distance, `d_dot` is
  // target_distance_dot, and `result` is the longitudinal acceleration returned
  // by IDM.
  double get_dadv() const { return -2 * a() * s_0() * time_headway(); }

  // Returns ∂/∂d(result) for the special case where v = d_dot = 0 and d = 1,
  // where `v` is ego_velocity, `d` is target_distance, `d_dot` is
  // target_distance_dot, and `result` is the longitudinal acceleration returned
  // by IDM.
  double get_dadd() const { return 2 * a() * s_0(); }

  // Sets the derivatives() to a 3-vector ordered as follows: {ego_velocity,
  // target_distance, target_distance_dot}.
  void SetAllDerivatives(T* ego_velocity, T* target_distance, T* target_distance_dot) {
    SetDerivatives(ego_velocity, Eigen::VectorXd::Unit(3, 0));
    SetDerivatives(target_distance, Eigen::VectorXd::Unit(3, 1));
    SetDerivatives(target_distance_dot, Eigen::VectorXd::Unit(3, 2));
  }

  double delta() const { return ExtractDoubleOrThrow(this->params_.delta()); }
  double a() const { return ExtractDoubleOrThrow(this->params_.a()); }
  double v_ref() const { return ExtractDoubleOrThrow(this->params_.v_ref()); }
  double s_0() const { return ExtractDoubleOrThrow(this->params_.s_0()); }
  double time_headway() const { return ExtractDoubleOrThrow(this->params_.time_headway()); }

  const IdmPlannerParameters<T> params_;
};

typedef ::testing::Types<double, AutoDiffXd> Implementations;
TYPED_TEST_CASE(IdmPlannerTest, Implementations);

// Set the initial states such that the agent and ego start at the headway
// distance, with the ego car closing in on the lead car.
TYPED_TEST(IdmPlannerTest, SameSpeedAtHeadwayDistance) {
  using T = TypeParam;

  T ego_velocity = this->params_.v_ref();
  T target_distance = this->params_.v_ref() * this->params_.time_headway();
  T target_distance_dot = -4 * sqrt(this->params_.a() * this->params_.b()) / this->params_.v_ref();
  this->SetAllDerivatives(&ego_velocity, &target_distance, &target_distance_dot);

  const T result = IdmPlanner<T>::Evaluate(this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect acceleration to be close to zero.
  EXPECT_NEAR(0., ExtractDoubleOrThrow(result), 1e-2);

  // Since (closing_term + too_close_term) = 0 in the IDM equation with these
  // values, it then follows that:
  //
  // ∂/∂v(result) ∝ ∂/∂v(accel_free_road) and
  // ∂/∂d(result) = ∂/∂d_dot(result) = 0.
  //
  // where v is ego_velocity, d = target_distance, and d_dot is
  // target_distance_dot.
  CheckDerivatives(result, Eigen::Vector3d{this->get_free_dadv(), 0., 0.});
}

// Set the initial states such that the agent and ego start within the headway
// distance, both at the desired speed.
TYPED_TEST(IdmPlannerTest, SameSpeedBelowHeadwayDistance) {
  using T = TypeParam;

  T ego_velocity = this->params_.v_ref();
  T target_distance = 6.;
  T target_distance_dot = 0.;
  this->SetAllDerivatives(&ego_velocity, &target_distance, &target_distance_dot);

  const T result = IdmPlanner<T>::Evaluate(this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to decelerate.
  EXPECT_GT(0., ExtractDoubleOrThrow(result));

  // Expect ∂/∂v(result) < 0, ∂/∂d(result) > 0, and ∂/∂d_dot(result) < 0.
  CheckDerivativeNegativity(0, result);
  CheckDerivativePositivity(1, result);
  CheckDerivativeNegativity(2, result);
}

// Set the initial states such that the agent and ego start close together at
// different speeds.
TYPED_TEST(IdmPlannerTest, DifferentSpeedsBelowHeadwayDistance) {
  using T = TypeParam;

  T ego_velocity = 7.;
  T target_distance = 6.;
  T target_distance_dot = 3.;
  this->SetAllDerivatives(&ego_velocity, &target_distance, &target_distance_dot);

  const T result = IdmPlanner<T>::Evaluate(this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to decelerate.
  EXPECT_GT(0., ExtractDoubleOrThrow(result));

  // Expect ∂/∂v(result) < 0, ∂/∂d(result) > 0, and ∂/∂d_dot(result) < 0.
  CheckDerivativeNegativity(0, result);
  CheckDerivativePositivity(1, result);
  CheckDerivativeNegativity(2, result);
}

// Set the agent and ego infinitely far apart from one another, with the ego car
// initially at the desired speed.
TYPED_TEST(IdmPlannerTest, EgoAtDesiredSpeed) {
  using T = TypeParam;

  T ego_velocity = this->params_.v_ref();
  T target_distance = std::numeric_limits<double>::infinity();
  T target_distance_dot = this->params_.v_ref();
  this->SetAllDerivatives(&ego_velocity, &target_distance, &target_distance_dot);

  const T result = IdmPlanner<T>::Evaluate(this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect acceleration to be close to zero.
  EXPECT_NEAR(0., ExtractDoubleOrThrow(result), 1e-2);

  // As above, since (closing_term + too_close_term) = 0 (to within tolerance),
  // the following result is immediate.
  CheckDerivatives(result, Eigen::Vector3d{this->get_free_dadv(), 0., 0.});
}

// Set the agent and ego sufficiently far apart from one another, with the ego
// car speed initially zero.
TYPED_TEST(IdmPlannerTest, EgoStartFromRest) {
  using T = TypeParam;

  T ego_velocity = 0.;
  T target_distance = 1.;
  T target_distance_dot = 0.;
  this->SetAllDerivatives(&ego_velocity, &target_distance, &target_distance_dot);

  const T result = IdmPlanner<T>::Evaluate(this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to stay at rest at this target distance.
  EXPECT_EQ(0., ExtractDoubleOrThrow(result));

  // Since v = d_dot = 0, then ∂/∂d_dot(result) = 0 and ∂/∂v(result) and
  // ∂/∂d(result) reduce to the simple relations provided above.
  CheckDerivatives(result, Eigen::Vector3d{this->get_dadv(), this->get_dadd(), 0.});
}

// Set the agent and ego sufficiently far apart from one another, with the ego
// car speed initially zero.
TYPED_TEST(IdmPlannerTest, EgoStartWithNegativeSpeed) {
  using T = TypeParam;

  T ego_velocity = -1.;
  T target_distance = 1.;
  T target_distance_dot = 0.;
  this->SetAllDerivatives(&ego_velocity, &target_distance, &target_distance_dot);

  const T result = IdmPlanner<T>::Evaluate(this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to accelerate.
  EXPECT_LT(0., ExtractDoubleOrThrow(result));

  // Expect ∂/∂v(result) < 0, ∂/∂d(result) > 0, and ∂/∂d_dot(result) > 0.
  CheckDerivativeNegativity(0, result);
  CheckDerivativePositivity(1, result);
  CheckDerivativePositivity(2, result);
}

}  // namespace
}  // namespace delphyne
