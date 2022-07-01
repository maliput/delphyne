// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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
#include "delphyne/roads/road_network_wrapper.h"

#include <memory>

#include <gtest/gtest.h>
#include <maliput/test_utilities/mock.h>

namespace delphyne {
namespace {

class RoadNetworkWrapperTest : public ::testing::Test {
 public:
  std::unique_ptr<maliput::api::RoadNetwork> road_network_ = maliput::api::test::CreateRoadNetwork();
};

TEST_F(RoadNetworkWrapperTest, Constructor) {
  // roads::RoadNetworkWrapper(nullptr);
  EXPECT_THROW(roads::RoadNetworkWrapper(nullptr), std::runtime_error);
  // roads::RoadNetworkWrapper dut
  EXPECT_NO_THROW(roads::RoadNetworkWrapper(std::move(road_network_)));
}

TEST_F(RoadNetworkWrapperTest, Operators) {
  roads::RoadNetworkWrapper dut(std::move(road_network_));
  EXPECT_NE(dut.operator->(), nullptr);
  EXPECT_NO_THROW(dut->road_geometry()->id());
  EXPECT_NO_THROW((*dut).road_geometry()->id());
}

TEST_F(RoadNetworkWrapperTest, Release) {
  roads::RoadNetworkWrapper dut(std::move(road_network_));
  auto raw_ptr = dut.release();
  EXPECT_EQ(dut.operator->(), nullptr);
  EXPECT_NE(raw_ptr, nullptr);
  EXPECT_NO_THROW(raw_ptr->road_geometry()->id());
  delete raw_ptr;
}

}  // namespace
}  // namespace delphyne
