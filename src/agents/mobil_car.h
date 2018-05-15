// Copyright 2017 Toyota Research Institute

#include <memory>

#include "../../include/delphyne/agent_plugin_base.h"

namespace delphyne {

class MobilCarAgentParams final : public delphyne::AgentPluginParams {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilCarAgentParams)

  explicit MobilCarAgentParams(bool initial_with_s)
      : initial_with_s_(initial_with_s) {}

  bool get_initial_with_s() { return initial_with_s_; }

 private:
  bool initial_with_s_{true};
};

}  // namespace delphyne
