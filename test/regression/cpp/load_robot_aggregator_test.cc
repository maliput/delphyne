// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include "backend/load_robot_aggregator.h"

#include <functional>
#include <memory>
#include <vector>

#include <drake/systems/framework/framework_common.h>
#include <gtest/gtest.h>

#include "test_utilities/helpers.h"

namespace delphyne {

using drake::lcmt_viewer_geometry_data;
using drake::lcmt_viewer_link_data;
using drake::lcmt_viewer_load_robot;

// @brief Checks that LCM viewer load robot messages on the input port of the
// system are correctly aggregated into a new load robot message.
GTEST_TEST(LoadRobotAggregatorSystemTest, TwoMessagesAggregation) {
  // Two load robot messages will be aggregated.
  lcmt_viewer_load_robot original = test::BuildPreloadedLoadRobotMsg();

  // The second messages is a slight variation on the first one:
  // it changes the robot_num field (i.e. the id of each model).
  lcmt_viewer_load_robot modified = original;
  for (lcmt_viewer_link_data& link : modified.link) {
    // robot_num is replaced with a value opposite to the original one.
    link.robot_num = -link.robot_num;
  }

  const LoadRobotAggregator aggregator({original, modified});
  std::unique_ptr<drake::systems::Context<double>> context = aggregator.AllocateContext();

  std::unique_ptr<drake::systems::SystemOutput<double>> output = aggregator.AllocateOutput();
  aggregator.CalcOutput(*context, output.get());

  const auto& aggregated_load_robot =
      output->get_data(LoadRobotAggregator::kPortIndex)->get_value<lcmt_viewer_load_robot>();

  // The resulting message should have twice the number of links as the original
  // message (since two of those were aggregated).
  ASSERT_EQ(original.num_links * 2, aggregated_load_robot.num_links);
  for (int i = 0; i < original.num_links; ++i) {
    const lcmt_viewer_link_data& original_link = original.link[i];
    const lcmt_viewer_link_data& unmodified_link = aggregated_load_robot.link[i];
    const lcmt_viewer_link_data& modified_link = aggregated_load_robot.link[i + original.num_links];

    EXPECT_EQ(original_link.robot_num, unmodified_link.robot_num);
    EXPECT_EQ(-original_link.robot_num, modified_link.robot_num);

    EXPECT_EQ(original_link.name, unmodified_link.name);
    EXPECT_EQ(original_link.name, modified_link.name);

    for (int j = 0; j < original.link[i].num_geom; ++j) {
      // TODO(nventuro): find a way of comparing two lcm messages without having
      // to go by hand through each field. Maybe encode() can be used?
      // EXPECT_EQ(0, memcmp(&original_link.geom[j], &unmodified_link.geom[j],
      // sizeof(lcmt_viewer_geometry_data)));
      // EXPECT_EQ(0, memcmp(&original_link.geom[j], &modified_link.geom[j],
      // sizeof(lcmt_viewer_geometry_data)));
    }
  }
}

}  // namespace delphyne
