// Copyright 2017 Toyota Research Institute

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

// An example class that derives from the AgentPluginAutoDiffXdBase (see
// agent_plugin_base.h for more information).  This simple class does nothing
// except for return success for all method calls.
class LoadableExampleAutoDiffXd final
    : public delphyne::AgentPluginAutoDiffXdBase {
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

// An example factory class that derives from AgentPluginFactoryAutoDiffXdBase
// (see agent_plugin_base.h for more information).  This factory creates and
// returns a std::unique_ptr of the LoadableExampleAutoDiffXd above, and
// showcases the way almost all loadable plugins should implement the factory
// class.
class LoadableExampleFactoryAutoDiffXd final
    : public delphyne::AgentPluginFactoryAutoDiffXdBase {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<::drake::AutoDiffXd>>
  Create() {
    return std::make_unique<LoadableExampleAutoDiffXd>();
  }
};

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    LoadableExampleFactoryAutoDiffXd,
    delphyne::AgentPluginFactoryAutoDiffXdBase)
