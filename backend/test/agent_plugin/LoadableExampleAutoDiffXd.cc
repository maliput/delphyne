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

#include <map>
#include <string>

#include <ignition/common/PluginMacros.hh>

#include "backend/linb-any"

#include "backend/agent_plugin_base.h"

#include "drake/automotive/car_vis_applicator.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/pose_aggregator.h"

class LoadableExampleAutoDiffXd final
    : public delphyne::backend::AgentPluginAutoDiffXdBase {
 public:
  int Configure(const std::map<std::string, linb::any>& parameters,
                drake::systems::DiagramBuilder<::drake::AutoDiffXd>* builder,
                drake::lcm::DrakeLcmInterface* lcm, const std::string& name,
                int id,
                drake::systems::rendering::PoseAggregator<::drake::AutoDiffXd>*
                    aggregator,
                drake::automotive::CarVisApplicator<::drake::AutoDiffXd>*
                    car_vis_applicator) override {
    return 0;
  }

  int Initialize(
      drake::systems::Context<::drake::AutoDiffXd>* context) override {
    return 0;
  }
};

class LoadableExampleFactoryAutoDiffXd final
    : public delphyne::backend::AgentPluginFactoryAutoDiffXdBase {
 public:
  std::unique_ptr<delphyne::backend::AgentPluginBase<::drake::AutoDiffXd>>
  Create() {
    return std::make_unique<LoadableExampleAutoDiffXd>();
  }
};

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    LoadableExampleFactoryAutoDiffXd,
    delphyne::backend::AgentPluginFactoryAutoDiffXdBase)
