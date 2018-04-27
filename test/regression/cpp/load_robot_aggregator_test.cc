// Copyright 2018 Toyota Research Institute

#include "backend/load_robot_aggregator.h"

#include <functional>
#include <memory>
#include <vector>

#include "drake/systems/framework/framework_common.h"

#include "gtest/gtest.h"

#include "test/regression/cpp/helpers.h"

namespace delphyne {

using drake::lcmt_viewer_load_robot;
using drake::lcmt_viewer_link_data;
using drake::lcmt_viewer_geometry_data;

// @brief Checks that LCM viewer load robot messages on the input port of the
// system are correctly aggregated into a new load robot message.
GTEST_TEST(LoadRobotAggregatorSystemTest, TwoMessagesAggregation) {
  using LoadRobotGenerator = std::function<lcmt_viewer_load_robot()>;
  std::vector<LoadRobotGenerator> load_robot_generators;

  // Two generators will return a load robot message based on an original
  // template message, modified slightly by each.
  lcmt_viewer_load_robot original = test::BuildPreloadedLoadRobotMsg();

  // The first generator does not modify the original message.
  load_robot_generators.push_back([&original]() { return original; });

  // The second generator changes the robot_num field (the id of each model).
  load_robot_generators.push_back([&original]() {
    lcmt_viewer_load_robot modified = original;
    for (lcmt_viewer_link_data& link : modified.link) {
      // robot_num is replaced with a value opposite to the original one.
      link.robot_num = -link.robot_num;
    }
    return modified;
  });

  const LoadRobotAggregator aggregator;
  std::unique_ptr<drake::systems::Context<double>> context =
      aggregator.AllocateContext();

  context->FixInputPort(
      LoadRobotAggregator::kPortIndex,
      drake::systems::AbstractValue::Make(load_robot_generators));

  std::unique_ptr<drake::systems::SystemOutput<double>> output =
      aggregator.AllocateOutput(*context);
  aggregator.CalcOutput(*context, output.get());

  const auto& aggregated_load_robot =
      output->get_data(LoadRobotAggregator::kPortIndex)
          ->GetValue<lcmt_viewer_load_robot>();

  // The resulting message should have twice the number of links as the original
  // message (since two of those were aggregated).
  ASSERT_EQ(original.num_links * 2, aggregated_load_robot.num_links);
  for (int i = 0; i < original.num_links; ++i) {
    const lcmt_viewer_link_data& original_link = original.link[i];
    const lcmt_viewer_link_data& unmodified_link =
        aggregated_load_robot.link[i];
    const lcmt_viewer_link_data& modified_link =
        aggregated_load_robot.link[i + original.num_links];

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
