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
#include "systems/simple_powertrain.h"

#include <memory>

#include <gtest/gtest.h>

#include "test_utilities/scalar_conversion.h"

using std::make_unique;

namespace delphyne {
namespace {

// Specify a gain and a time constant for the lag model.
static constexpr double kPowertrainTimeConstant{0.2}; /* [s] */
static constexpr double kPowertrainGain{5.};          /* [N] */

class SimplePowertrainTest : public ::testing::Test {
 protected:
  void SetUp() override { dut_.reset(new SimplePowertrain<double>(kPowertrainTimeConstant, kPowertrainGain)); }
  std::unique_ptr<SimplePowertrain<double>> dut_;  //< The device under test.
};

// Verifies the supplied data can be accessed.
TEST_F(SimplePowertrainTest, Accessors) {
  EXPECT_EQ(kPowertrainTimeConstant, dut_->get_time_constant());
  EXPECT_EQ(kPowertrainGain, dut_->get_gain());
}

// Verifies the correctness of the model.
TEST_F(SimplePowertrainTest, SystemMatrices) {
  // Check the properties of the system and the coefficients of the state and
  // output equations.
  EXPECT_FALSE(dut_->HasAnyDirectFeedthrough());
  EXPECT_EQ(drake::Vector1<double>(-1. / kPowertrainTimeConstant), dut_->A());
  EXPECT_EQ(drake::Vector1<double>(kPowertrainGain), dut_->B());
  EXPECT_EQ(drake::Vector1<double>(1. / kPowertrainTimeConstant), dut_->C());
}

// clang-format off
TEST_F(SimplePowertrainTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_));
}

TEST_F(SimplePowertrainTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*dut_));
}
// clang-format on

}  // namespace
}  // namespace delphyne
