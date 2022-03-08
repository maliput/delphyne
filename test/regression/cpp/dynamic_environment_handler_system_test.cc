#include "backend/dynamic_environment_handler_system.h"

#include <memory>

#include <drake/systems/analysis/simulator.h>
#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/mock.h>

#include "backend/dynamic_environment_handler.h"

namespace delphyne {
namespace {

class MockDynamicEnvironmentHandler : public DynamicEnvironmentHandler {
 public:
  MockDynamicEnvironmentHandler(maliput::api::RoadNetwork* road_network) : DynamicEnvironmentHandler(road_network) {}

  void Update(double sim_time) override { sim_time_ = sim_time; }
  double sim_time_{0.};
};

class DynamicEnvironmentHandlerSystemTest : public ::testing::Test {
 public:
  static constexpr double kPhaseDuration{1.};
  void SetUp() override { ASSERT_NE(rn_, nullptr); }

  std::unique_ptr<maliput::api::RoadNetwork> rn_ = maliput::api::test::CreateRoadNetwork();
};

TEST_F(DynamicEnvironmentHandlerSystemTest, System) {
  auto mock_deh = std::make_unique<MockDynamicEnvironmentHandler>(rn_.get());
  auto mock_deh_ptr = mock_deh.get();
  DynamicEnvironmentHandlerSystem dut{std::move(mock_deh)};

  // Creates a simulator to work with the publisher.
  drake::systems::Simulator<double> simulator(dut, dut.CreateDefaultContext());

  // Simulates for a small time period.
  const double kStartTime{0.};
  const double kFirstTime{1.};
  const double kSecondTime{2.};
  simulator.Initialize();
  simulator.AdvanceTo(kFirstTime);
  // As the class is using DeclarePerStepUnrestrictedUpdateEvent method it is expected to be executed before stepping
  // up.
  EXPECT_EQ(kStartTime, mock_deh_ptr->sim_time_);
  simulator.AdvanceTo(kSecondTime);
  EXPECT_EQ(kFirstTime, mock_deh_ptr->sim_time_);
}

}  // namespace
}  // namespace delphyne
