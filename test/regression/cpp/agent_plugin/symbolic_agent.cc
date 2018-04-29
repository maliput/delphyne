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

// An example class that derives from the AgentPluginExpressionBase (see
// agent_plugin_base.h for more information).  This simple class does nothing
// except for return success for all method calls.
class SymbolicAgent final : public delphyne::SymbolicAgentPlugin {
 public:
  int Configure(
      const std::map<std::string, linb::any>& parameters,
      drake::systems::DiagramBuilder<delphyne::Symbolic>* builder,
      drake::lcm::DrakeLcmInterface* lcm, const std::string& name, int id,
      drake::systems::rendering::PoseAggregator<delphyne::Symbolic>*
          aggregator,
      drake::automotive::CarVisApplicator<delphyne::Symbolic>*
          car_vis_applicator) override {
    return 0;
  }

  int Initialize(drake::systems::Context<delphyne::Symbolic>*
                     context) override {
    return 0;
  }
};

// An example factory class that derives from AgentPluginFactoryExpressionBase
// (see agent_plugin_base.h for more information).  This factory creates and
// returns a std::unique_ptr of the SymbolicAgent above, and
// showcases the way almost all loadable plugins should implement the factory
// class.
class LoadableExampleFactoryExpression final
    : public delphyne::SymbolicAgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<delphyne::Symbolic>>
  Create() {
    return std::make_unique<SymbolicAgent>();
  }
};

IGN_COMMON_REGISTER_SINGLE_PLUGIN(LoadableExampleFactoryExpression,
                                  delphyne::SymbolicAgentPluginFactory)
