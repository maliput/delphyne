// Copyright 2017 Toyota Research Institute

#include "delphyne/mi6/agent_diagram_builder.h"

#include <memory>

#include <gtest/gtest.h>

#include "helpers.h"
#include "systems/lane_direction.h"
#include "systems/simple_car.h"
#include "test/test_config.h"

namespace delphyne {

/*****************************************************************************
 * Tests
 ****************************************************************************/

// Fixture class for share configuration among all tests.
// Define Setup() if you need to set env variables and the like
//
//                         -----------
//                        |           | --> State
//      Traffic Poses --> |  Diagram  | --> Pose
//                        |           | --> Velocity
//                         -----------
//
struct AgentDiagramBuilderTest : public ::testing::Test {
  using SimpleCarSystem = SimpleCar2<double>;

  AgentDiagramBuilderTest()
      : builder("foo"),
        system(builder.AddSystem(std::make_unique<SimpleCarSystem>())) {}
  AgentDiagramBuilder<double> builder;
  SimpleCarSystem* system{};
};

TEST_F(AgentDiagramBuilderTest, BuildWithoutExports) {
  EXPECT_THROW_OF_TYPE(std::runtime_error, builder.Build(),
                       "A state output port has not been exported (see "
                       "AgentDiagramBuilder::ExportStateOutput)");
}

TEST_F(AgentDiagramBuilderTest, BuildWithTooManyExports) {
  builder.ExportStateOutput(system->state_output());
  EXPECT_THROW_OF_TYPE(
      std::runtime_error, builder.ExportStateOutput(system->state_output()),
      "A state output port has already been exported and this diagram "
      "enforces that there can be only one.");
  builder.ExportPoseOutput(system->pose_output());
  EXPECT_THROW_OF_TYPE(
      std::runtime_error, builder.ExportPoseOutput(system->pose_output()),
      "A pose output port has already been exported and this diagram "
      "enforces that there can be only one.");
  builder.ExportVelocityOutput(system->velocity_output());
  EXPECT_THROW_OF_TYPE(
      std::runtime_error,
      builder.ExportVelocityOutput(system->velocity_output()),
      "A velocity output port has already been exported and this diagram "
      "enforces that there can be only one.");
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace delphyne
