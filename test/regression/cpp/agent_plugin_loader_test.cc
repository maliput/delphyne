// Copyright 2017 Toyota Research Institute

#include "backend/agent_plugin_loader.h"

#include "drake/systems/framework/diagram_builder.h"

#include <stdlib.h>

#include <gtest/gtest.h>

TEST(AgentPluginLoader, Invalid) {
  auto agent = delphyne::LoadPlugin<double>("foo");
  ASSERT_EQ(nullptr, agent);
}

static const char* env = "DELPHYNE_AGENT_PLUGIN_PATH=agent_plugin";

TEST(AgentPluginLoader, ExampleDouble) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::LoadPlugin<double>("LoadableExampleDouble");
  ASSERT_NE(nullptr, agent);

  // We construct and use a drake DiagramBuilder here just to ensure that we
  // have at least one symbol referencing libdrake.so so the linker properly
  // links it in.  Otherwise, when the modules attempt to load, the drake
  // symbols are missing and they fail to load.
  std::unique_ptr<drake::systems::DiagramBuilder<double>> builder{
      std::make_unique<drake::systems::DiagramBuilder<double>>()};

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure(params, builder.get(), nullptr, "testname", 0,
                                nullptr, nullptr));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}

TEST(AgentPluginLoader, ExampleAutodiff) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::LoadPlugin<::drake::AutoDiffXd>(
      "LoadableExampleAutoDiffXd");
  ASSERT_NE(nullptr, agent);

  // We construct and use a drake DiagramBuilder here just to ensure that we
  // have at least one symbol referencing libdrake.so so the linker properly
  // links it in.  Otherwise, when the modules attempt to load, the drake
  // symbols are missing and they fail to load.
  std::unique_ptr<drake::systems::DiagramBuilder<::drake::AutoDiffXd>> builder{
      std::make_unique<drake::systems::DiagramBuilder<::drake::AutoDiffXd>>()};

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure(params, builder.get(), nullptr, "testname", 0,
                                nullptr, nullptr));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}

TEST(AgentPluginLoader, ExampleSymbolic) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::LoadPlugin<::drake::symbolic::Expression>(
      "LoadableExampleExpression");
  ASSERT_NE(nullptr, agent);

  // We construct and use a drake DiagramBuilder here just to ensure that we
  // have at least one symbol referencing libdrake.so so the linker properly
  // links it in.  Otherwise, when the modules attempt to load, the drake
  // symbols are missing and they fail to load.
  std::unique_ptr<drake::systems::DiagramBuilder<::drake::symbolic::Expression>>
      builder{std::make_unique<
          drake::systems::DiagramBuilder<::drake::symbolic::Expression>>()};

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure(params, builder.get(), nullptr, "testname", 0,
                                nullptr, nullptr));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}
