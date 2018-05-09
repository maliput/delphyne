// Copyright 2017 Toyota Research Institute

#include <stdlib.h>

#include <gtest/gtest.h>
#include "drake/systems/framework/diagram_builder.h"

#include "../../../include/delphyne/types.h"
#include "backend/agent_plugin_loader.h"

TEST(AgentPluginLoader, Invalid) {
  auto agent = delphyne::LoadPlugin<double>("foo");
  ASSERT_EQ(nullptr, agent);
}

static const char* env = "DELPHYNE_AGENT_PLUGIN_PATH=agent_plugin";

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

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure("testname", 0, params, builder.get(), nullptr,
                                nullptr));

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

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure("testname", 0, params, builder.get(), nullptr,
                                nullptr));

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

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure(params, builder.get(), "testname", 0, nullptr,
                                nullptr));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}
