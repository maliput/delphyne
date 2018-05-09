// Copyright 2017 Toyota Research Institute

#include <memory>

#include "include/delphyne/agent_plugin_base.h"

namespace delphyne {

/// This class models the required extra parameters to create a mobil car.
class MobilCarAgentParams final : public delphyne::AgentPluginParams {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilCarAgentParams)

  /// Default constructor.
  ///
  /// @param[in] initial_with_s Initial travel direction in the lane. (See
  /// MobilPlanner documentation).
  explicit MobilCarAgentParams(bool initial_with_s)
      : initial_with_s_(initial_with_s) {}

  /// Returns the initial travel direction.
  bool get_initial_with_s() const { return initial_with_s_; }

 private:
  bool initial_with_s_;
};

}  // namespace delphyne
