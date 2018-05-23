// Copyright 2017 Toyota Research Institute

#include <stdlib.h>

#include <gtest/gtest.h>
#include "drake/systems/framework/diagram_builder.h"

#include "backend/agent_plugin_loader.h"
#include "include/delphyne/agent_plugin_base.h"
#include "include/delphyne/types.h"

namespace delphyne {
namespace {

// Type parameterized test suite for agent plugin loading.
template <typename T>
class AgentPluginLoaderTest : public ::testing::Test {
 protected:
  AgentPluginLoaderTest()
      // We construct and use a drake DiagramBuilder here just to ensure that we
      // have at least one symbol referencing libdrake.so so the linker properly
      // links it in.  Otherwise, when the modules attempt to load, the drake
      // symbols are missing and they fail to load.
      : builder{std::make_unique<drake::systems::DiagramBuilder<T>>()} {}

  // Returns the name of the plugin library to load the agent from.
  // @note To be explicitly specialized on a per base type basis.
  static const char* GetPluginLibraryName();

  // A diagram builder instance.
  const std::unique_ptr<drake::systems::DiagramBuilder<T>> builder;
};

template <>
const char* AgentPluginLoaderTest<double>::GetPluginLibraryName() {
  return "simple-agent";
}

template <>
const char* AgentPluginLoaderTest<Symbolic>::GetPluginLibraryName() {
  return "symbolic-agent";
}

template <>
const char* AgentPluginLoaderTest<AutoDiff>::GetPluginLibraryName() {
  return "auto-diff-agent";
}

typedef ::testing::Types<double, AutoDiff, Symbolic> AgentBaseTypes;

TYPED_TEST_CASE(AgentPluginLoaderTest, AgentBaseTypes);

// Tests that agent plugin loading fails on a non-existent plugin library..
TYPED_TEST(AgentPluginLoaderTest, InvalidAgentPlugin) {
  auto agent = LoadPlugin<TypeParam>("foo");
  ASSERT_EQ(nullptr, agent);
}

// Tests that an agent can be properly loaded, configured and initialized.
TYPED_TEST(AgentPluginLoaderTest, AgentInitialization) {
  const int kSuccessReturnCode{0};
  const char* const kEnvironment{"DELPHYNE_AGENT_PLUGIN_PATH=agent_plugin"};
  ASSERT_EQ(kSuccessReturnCode, putenv(const_cast<char*>(kEnvironment)));
  auto agent = LoadPlugin<TypeParam>(
      AgentPluginLoaderTest<TypeParam>::GetPluginLibraryName());
  ASSERT_NE(nullptr, agent);
  auto params = std::make_unique<AgentPluginParams>();

  const int kDefaultId{0};
  drake::automotive::CarVisApplicator<TypeParam>* const kNullCarVisApplicator{
      nullptr};
  drake::systems::rendering::PoseAggregator<TypeParam>* const
      kNullPoseAggregator{nullptr};
  drake::maliput::api::RoadGeometry* const kNullRoad{nullptr};
  ASSERT_EQ(
      kSuccessReturnCode,
      agent->Configure("agent-initialization", kDefaultId, this->builder.get(),
                       kNullPoseAggregator, kNullCarVisApplicator, kNullRoad,
                       std::move(params)));
  ASSERT_EQ(kSuccessReturnCode, agent->Initialize(nullptr));
}

}  // namespace
}  // namespace delphyne
