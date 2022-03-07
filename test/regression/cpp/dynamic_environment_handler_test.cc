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
