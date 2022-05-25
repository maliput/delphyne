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
#include "backend/dynamic_environment_handler.h"

#include <memory>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/mock.h>

namespace delphyne {
namespace {

class MockDynamicEnvironmentHandler : public DynamicEnvironmentHandler {
 public:
  MockDynamicEnvironmentHandler(maliput::api::RoadNetwork* road_network) : DynamicEnvironmentHandler(road_network) {}

  void Update(double sim_time) override { sim_time_ = sim_time; }
  double sim_time_{0.};
};

class DynamicEnvironmentHandlerTest : public ::testing::Test {
 public:
  void SetUp() override { ASSERT_NE(rn_, nullptr); }

  std::unique_ptr<maliput::api::RoadNetwork> rn_ = maliput::api::test::CreateRoadNetwork();
};

TEST_F(DynamicEnvironmentHandlerTest, Constructor) {
  EXPECT_THROW(MockDynamicEnvironmentHandler(nullptr), std::invalid_argument);
  EXPECT_NO_THROW(MockDynamicEnvironmentHandler(rn_.get()));
}

TEST_F(DynamicEnvironmentHandlerTest, API) {
  MockDynamicEnvironmentHandler mock_deh{rn_.get()};
  DynamicEnvironmentHandler* dut = dynamic_cast<DynamicEnvironmentHandler*>(&mock_deh);
  const double kSimTime{5.};
  // Update
  dut->Update(kSimTime /* arbitrary sim time */);
  EXPECT_DOUBLE_EQ(kSimTime, mock_deh.sim_time_);
}

}  // namespace
}  // namespace delphyne
