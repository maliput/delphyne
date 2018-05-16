// Copyright 2017 Toyota Research Institute

#include <stdlib.h>

#include <gtest/gtest.h>
#include "drake/systems/framework/diagram_builder.h"

#include "backend/agent_plugin_loader.h"
#include "include/delphyne/agent_plugin_base.h"
#include "include/delphyne/types.h"

TEST(AgentPluginLoader, Invalid) {
  auto agent = delphyne::LoadPlugin<double>("foo");
  ASSERT_EQ(nullptr, agent);
}

static const char* env = "DELPHYNE_AGENT_PLUGIN_PATH=agent_plugin";
static const drake::maliput::api::RoadGeometry* kNullRoad{nullptr};

TEST(AgentPluginLoader, SimpleAgent) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::LoadPlugin<double>("simple-agent");
  ASSERT_NE(nullptr, agent);

  // We construct and use a drake DiagramBuilder here just to ensure that we
  // have at least one symbol referencing libdrake.so so the linker properly
  // links it in.  Otherwise, when the modules attempt to load, the drake
  // symbols are missing and they fail to load.
  std::unique_ptr<drake::systems::DiagramBuilder<double>> builder{
      std::make_unique<drake::systems::DiagramBuilder<double>>()};

  drake::systems::rendering::PoseAggregator<double>* kNullPoseAggregator{
      nullptr};
  drake::automotive::CarVisApplicator<double>* kNullCarVisApplicator{nullptr};

  auto params = std::make_unique<delphyne::AgentPluginParams>();
  ASSERT_EQ(
      0, agent->Configure("testname", 0, builder.get(), kNullPoseAggregator,
                          kNullCarVisApplicator, kNullRoad, std::move(params)));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}

TEST(AgentPluginLoader, AutoDiffAgent) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::LoadPlugin<delphyne::AutoDiff>("auto-diff-agent");
  ASSERT_NE(nullptr, agent);

  // We construct and use a drake DiagramBuilder here just to ensure that we
  // have at least one symbol referencing libdrake.so so the linker properly
  // links it in.  Otherwise, when the modules attempt to load, the drake
  // symbols are missing and they fail to load.
  std::unique_ptr<drake::systems::DiagramBuilder<delphyne::AutoDiff>> builder{
      std::make_unique<drake::systems::DiagramBuilder<delphyne::AutoDiff>>()};

  drake::systems::rendering::PoseAggregator<delphyne::AutoDiff>*
      kNullPoseAggregator{nullptr};
  drake::automotive::CarVisApplicator<delphyne::AutoDiff>*
      kNullCarVisApplicator{nullptr};

  auto params = std::make_unique<delphyne::AgentPluginParams>();
  ASSERT_EQ(
      0, agent->Configure("testname", 0, builder.get(), kNullPoseAggregator,
                          kNullCarVisApplicator, kNullRoad, std::move(params)));
  ASSERT_EQ(0, agent->Initialize(nullptr));
}

TEST(AgentPluginLoader, SymbolicAgent) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::LoadPlugin<delphyne::Symbolic>("symbolic-agent");
  ASSERT_NE(nullptr, agent);

  // We construct and use a drake DiagramBuilder here just to ensure that we
  // have at least one symbol referencing libdrake.so so the linker properly
  // links it in.  Otherwise, when the modules attempt to load, the drake
  // symbols are missing and they fail to load.
  std::unique_ptr<drake::systems::DiagramBuilder<delphyne::Symbolic>> builder{
      std::make_unique<drake::systems::DiagramBuilder<delphyne::Symbolic>>()};

  drake::systems::rendering::PoseAggregator<delphyne::Symbolic>*
      kNullPoseAggregator{nullptr};
  drake::automotive::CarVisApplicator<delphyne::Symbolic>*
      kNullCarVisApplicator{nullptr};

  auto params = std::make_unique<delphyne::AgentPluginParams>();
  ASSERT_EQ(
      0, agent->Configure("testname", 0, builder.get(), kNullPoseAggregator,
                          kNullCarVisApplicator, kNullRoad, std::move(params)));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}
