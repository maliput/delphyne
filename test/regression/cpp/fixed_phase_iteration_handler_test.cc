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
#include "backend/fixed_phase_iteration_handler.h"

#include <memory>

#include <delphyne/roads/road_builder.h>
#include <gtest/gtest.h>
#include <maliput/api/intersection.h>
#include <maliput/api/intersection_book.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/phase.h>
#include <maliput/common/assertion_error.h>
#include <maliput/common/filesystem.h>

namespace delphyne {
namespace {

constexpr char MALIPUT_MALIDRIVE_RESOURCE_VAR[] = "MALIPUT_MALIDRIVE_RESOURCE_ROOT";

// Uses maliput_malidrive's SingleRoadPedestrianCrosswalk phase rings to evaluate the FixedPhaseIterationHandler
// implementation.
class FixedPhaseIterationHandlerTest : public ::testing::Test {
 public:
  static constexpr char kYamlFileName[] = "/resources/odr/SingleRoadPedestrianCrosswalk.yaml";
  static constexpr char kXodrFileName[] = "/resources/odr/SingleRoadPedestrianCrosswalk.xodr";
  const std::string kXodrFilePath{maliput::common::Filesystem::get_env_path(MALIPUT_MALIDRIVE_RESOURCE_VAR) +
                                  kXodrFileName};
  const std::string kYamlFilePath{maliput::common::Filesystem::get_env_path(MALIPUT_MALIDRIVE_RESOURCE_VAR) +
                                  kYamlFileName};
  const double kPhaseDuration{0.5};

  void SetUp() override {
    rn_ = roads::CreateMalidriveRoadNetworkFromXodr("test", kXodrFilePath, kYamlFilePath, kYamlFilePath, kYamlFilePath,
                                                    kYamlFilePath, kYamlFilePath);
  }

  std::unique_ptr<delphyne::roads::RoadNetworkWrapper> rn_;
};

TEST_F(FixedPhaseIterationHandlerTest, Constructor) {
  EXPECT_THROW(FixedPhaseIterationHandler(rn_->operator->(), -5.), std::invalid_argument);
  EXPECT_THROW(FixedPhaseIterationHandler(nullptr, 2.), std::invalid_argument);
  EXPECT_NO_THROW(FixedPhaseIterationHandler(rn_->operator->(), kPhaseDuration));
}

TEST_F(FixedPhaseIterationHandlerTest, VerifyPhasesBeingIterated) {
  const maliput::api::rules::Phase::Id kAllGoPhase{"AllGoPhase"};
  const maliput::api::rules::Phase::Id kAllStopPhase{"AllStopPhase"};

  // Obtains the intersection to be used for the analysis.
  maliput::api::IntersectionBook* intersection_book = (*rn_)->intersection_book();
  ASSERT_NE(intersection_book, nullptr);
  const maliput::api::Intersection* intersection =
      intersection_book->GetIntersection(maliput::api::Intersection::Id("PedestrianCrosswalkIntersection"));
  ASSERT_NE(intersection, nullptr);

  FixedPhaseIterationHandler dut{rn_->operator->(), kPhaseDuration};
  double sim_time = 0.;
  dut.Update(sim_time);
  // According to the IntersectionBook yaml file the initial phase is: AllGoPhase.
  EXPECT_EQ(kAllGoPhase, intersection->Phase()->state);

  sim_time = kPhaseDuration + kPhaseDuration / 2.;
  dut.Update(sim_time);
  // As the simulation time increases, the phase ring changes its state.
  EXPECT_EQ(kAllStopPhase, intersection->Phase()->state);
}

TEST_F(FixedPhaseIterationHandlerTest, GettersAndSetters) {
  const double kNewPhaseDuration{123.};
  FixedPhaseIterationHandler dut{rn_->operator->(), kPhaseDuration};
  dut.set_phase_duration(kNewPhaseDuration);
  EXPECT_EQ(kNewPhaseDuration, dut.get_phase_duration());
}

TEST_F(FixedPhaseIterationHandlerTest, CheckServiceForChangingPhaseDuration) {
  const double kNewPhaseDuration{123.};
  FixedPhaseIterationHandler dut{rn_->operator->(), kPhaseDuration};

  ignition::transport::Node node;
  ignition::msgs::Double new_phase_duration_req;
  new_phase_duration_req.set_data(kNewPhaseDuration);
  ASSERT_TRUE(node.Request(FixedPhaseIterationHandler::kSetPhaseDurationSrvName, new_phase_duration_req));

  EXPECT_EQ(kNewPhaseDuration, dut.get_phase_duration());
}

}  // namespace
}  // namespace delphyne
