// Copyright 2017 Toyota Research Institute

#include "backend/agent_plugin_loader.h"

#include <stdlib.h>

#include "gtest/gtest.h"

TEST(AgentPluginLoader, Invalid) {
  auto agent = delphyne::backend::LoadPlugin<double>("foo");
  ASSERT_EQ(nullptr, agent);
}

static const char* env = "DELPHYNE_AGENT_PLUGIN_PATH=test/agent_plugin";

TEST(AgentPluginLoader, ExampleDouble) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::backend::LoadPlugin<double>("LoadableExampleDouble");
  ASSERT_NE(nullptr, agent);

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure(params, nullptr, nullptr, "testname", 0,
                                nullptr, nullptr));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}

TEST(AgentPluginLoader, ExampleAutodiff) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::backend::LoadPlugin<::drake::AutoDiffXd>(
      "LoadableExampleAutoDiffXd");
  ASSERT_NE(nullptr, agent);

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure(params, nullptr, nullptr, "testname", 0,
                                nullptr, nullptr));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}

TEST(AgentPluginLoader, ExampleSymbolic) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto agent = delphyne::backend::LoadPlugin<::drake::symbolic::Expression>(
      "LoadableExampleExpression");
  ASSERT_NE(nullptr, agent);

  const std::map<std::string, linb::any> params;
  ASSERT_EQ(0, agent->Configure(params, nullptr, nullptr, "testname", 0,
                                nullptr, nullptr));

  ASSERT_EQ(0, agent->Initialize(nullptr));
}
