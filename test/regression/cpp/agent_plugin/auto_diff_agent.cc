// Copyright 2017 Toyota Research Institute

#include <map>
#include <string>

#include <ignition/common/PluginMacros.hh>

#include "drake/automotive/car_vis_applicator.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/pose_aggregator.h"

#include "../../../../include/delphyne/agent_plugin_base.h"
#include "../../../../include/delphyne/linb-any"

// An example class that derives from the AgentPluginAutoDiffXdBase (see
// agent_plugin_base.h for more information).  This simple class does nothing
// except for return success for all method calls.
class AutoDiffAgent final : public delphyne::AutoDiffAgentPlugin {
 public:
  int Configure(
      const std::string& name, int id,
      drake::systems::DiagramBuilder<delphyne::AutoDiff>* builder,
      drake::systems::rendering::PoseAggregator<delphyne::AutoDiff>* aggregator,
      drake::automotive::CarVisApplicator<delphyne::AutoDiff>*
          car_vis_applicator,
      const drake::maliput::api::RoadGeometry* road,
      std::unique_ptr<delphyne::AgentPluginParams> parameters) override {
    return 0;
  }

  int Initialize(
      drake::systems::Context<delphyne::AutoDiff>* context) override {
    return 0;
  }

  drake::systems::System<delphyne::AutoDiff>* get_system() const {
    return nullptr;
  }
};

// An example factory class that derives from AgentPluginFactoryAutoDiffXdBase
// (see agent_plugin_base.h for more information).  This factory creates and
// returns a std::unique_ptr of the AutoDiffAgent above, and
// showcases the way almost all loadable plugins should implement the factory
// class.
class AutoDiffAgentFactory final : public delphyne::AutoDiffAgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<delphyne::AutoDiff>> Create() {
    return std::make_unique<AutoDiffAgent>();
  }
};

IGN_COMMON_REGISTER_SINGLE_PLUGIN(AutoDiffAgentFactory,
                                  delphyne::AutoDiffAgentPluginFactory)
