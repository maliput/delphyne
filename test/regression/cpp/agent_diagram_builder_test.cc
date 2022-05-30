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

#include "delphyne/mi6/agent_diagram_builder.h"

#include <memory>

#include <gtest/gtest.h>

#include "systems/lane_direction.h"
#include "systems/simple_car.h"
#include "test/test_config.h"
#include "test_utilities/helpers.h"

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

  AgentDiagramBuilderTest() : builder("foo"), system(builder.AddSystem(std::make_unique<SimpleCarSystem>())) {}
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
  EXPECT_THROW_OF_TYPE(std::runtime_error, builder.ExportStateOutput(system->state_output()),
                       "A state output port has already been exported and this diagram "
                       "enforces that there can be only one.");
  builder.ExportPoseOutput(system->pose_output());
  EXPECT_THROW_OF_TYPE(std::runtime_error, builder.ExportPoseOutput(system->pose_output()),
                       "A pose output port has already been exported and this diagram "
                       "enforces that there can be only one.");
  builder.ExportVelocityOutput(system->velocity_output());
  EXPECT_THROW_OF_TYPE(std::runtime_error, builder.ExportVelocityOutput(system->velocity_output()),
                       "A velocity output port has already been exported and this diagram "
                       "enforces that there can be only one.");
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace delphyne
