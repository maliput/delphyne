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
#include "systems/driving_command_mux.h"

#include <memory>

#include <drake/common/autodiff.h>
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>
#include <gtest/gtest.h>

#include "gen/driving_command.h"
#include "test_utilities/scalar_conversion.h"

namespace delphyne {

using drake::AutoDiffXd;

namespace {

class DrivingCommandMuxTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mux_ = std::make_unique<DrivingCommandMux<double>>();
    context_ = mux_->CreateDefaultContext();
    output_ = mux_->AllocateOutput();
  }

  std::unique_ptr<DrivingCommandMux<double>> mux_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::SystemOutput<double>> output_;
};

TEST_F(DrivingCommandMuxTest, Basic) {
  // Confirm the shape.
  ASSERT_EQ(2, mux_->num_input_ports());
  ASSERT_EQ(1, mux_->steering_input().size());
  ASSERT_EQ(1, mux_->acceleration_input().size());
  ASSERT_EQ(1, mux_->num_output_ports());
  ASSERT_EQ(2, mux_->get_output_port(0).size());

  // Confirm the output is indeed a DrivingCommand.
  const DrivingCommand<double>* driving_command_output =
      dynamic_cast<const DrivingCommand<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, driving_command_output);

  // Provide input data.
  context_->FixInputPort(mux_->steering_input().get_index(),
                         drake::Value<drake::systems::BasicVector<double>>(drake::systems::BasicVector<double>{42.}));
  context_->FixInputPort(mux_->acceleration_input().get_index(),
                         drake::Value<drake::systems::BasicVector<double>>(drake::systems::BasicVector<double>{11.}));

  // Confirm output data.
  mux_->CalcOutput(*context_, output_.get());
  ASSERT_EQ(42., driving_command_output->steering_angle());
  ASSERT_EQ(11., driving_command_output->acceleration());
}

// clang-format off
TEST_F(DrivingCommandMuxTest, IsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state().size());
}
// clang-format on

// Tests conversion to AutoDiffXd.
TEST_F(DrivingCommandMuxTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*mux_, [&](const auto& converted) {
    EXPECT_EQ(2, converted.num_input_ports());
    EXPECT_EQ(1, converted.num_output_ports());

    EXPECT_EQ(1, converted.get_input_port(0).size());
    EXPECT_EQ(1, converted.get_input_port(1).size());
    EXPECT_EQ(2, converted.get_output_port(0).size());

    const auto context = converted.CreateDefaultContext();
    const auto output = converted.AllocateOutput();
    const DrivingCommand<AutoDiffXd>* driving_command_output =
        dynamic_cast<const DrivingCommand<AutoDiffXd>*>(output->get_vector_data(0));
    EXPECT_NE(nullptr, driving_command_output);
  }));
}

// Tests conversion to drake::symbolic::Expression.
TEST_F(DrivingCommandMuxTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*mux_, [&](const auto& converted) {
    EXPECT_EQ(2, converted.num_input_ports());
    EXPECT_EQ(1, converted.num_output_ports());

    EXPECT_EQ(1, converted.get_input_port(0).size());
    EXPECT_EQ(1, converted.get_input_port(1).size());
    EXPECT_EQ(2, converted.get_output_port(0).size());

    const auto context = converted.CreateDefaultContext();
    const auto output = converted.AllocateOutput();
    const DrivingCommand<drake::symbolic::Expression>* driving_command_output =
        dynamic_cast<const DrivingCommand<drake::symbolic::Expression>*>(output->get_vector_data(0));
    EXPECT_NE(nullptr, driving_command_output);
  }));
}

}  // namespace
}  // namespace delphyne
