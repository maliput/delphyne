// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
