#include "backend/fixed_phase_iteration_handler.h"

#include <memory>

#include <gtest/gtest.h>
#include <maliput/api/intersection.h>
#include <maliput/api/intersection_book.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/phase.h>
#include <maliput/common/assertion_error.h>
#include <maliput/common/filesystem.h>

#include <delphyne/roads/road_builder.h>

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

  std::unique_ptr<maliput::api::RoadNetwork> rn_;
};

TEST_F(FixedPhaseIterationHandlerTest, Constructor) {
  EXPECT_THROW(FixedPhaseIterationHandler(rn_.get(), -5.), std::invalid_argument);
  EXPECT_THROW(FixedPhaseIterationHandler(nullptr, 2.), std::invalid_argument);
  EXPECT_NO_THROW(FixedPhaseIterationHandler(rn_.get(), kPhaseDuration));
}

TEST_F(FixedPhaseIterationHandlerTest, VerifyPhasesBeingIterated) {
  const maliput::api::rules::Phase::Id kAllGoPhase{"AllGoPhase"};
  const maliput::api::rules::Phase::Id kAllStopPhase{"AllStopPhase"};

  // Obtains the intersection to be used for the analysis.
  maliput::api::IntersectionBook* intersection_book = rn_->intersection_book();
  ASSERT_NE(intersection_book, nullptr);
  const maliput::api::Intersection* intersection =
      intersection_book->GetIntersection(maliput::api::Intersection::Id("PedestrianCrosswalkIntersection"));
  ASSERT_NE(intersection, nullptr);

  FixedPhaseIterationHandler dut{rn_.get(), kPhaseDuration};
  double sim_time = 0.;
  dut.Update(sim_time);
  // According to the IntersectionBook yaml file the initial phase is: AllGoPhase.
  EXPECT_EQ(kAllGoPhase, intersection->Phase()->state);

  sim_time = kPhaseDuration + kPhaseDuration / 2.;
  dut.Update(sim_time);
  // As the simulation time increases, the phase ring changes its state.
  EXPECT_EQ(kAllStopPhase, intersection->Phase()->state);
}

}  // namespace
}  // namespace delphyne
