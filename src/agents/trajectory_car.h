// Copyright 2017 Toyota Research Institute

#include <memory>
#include <utility>

#include "drake/automotive/curve2.h"

#include "include/delphyne/agent_plugin_base.h"

namespace delphyne {

class TrajectoryCarAgentParams final : public delphyne::AgentPluginParams {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryCarAgentParams)

  /// Default constructor.
  ///
  /// @param[in] curve The curve the car has to follow.
  explicit TrajectoryCarAgentParams(
      std::unique_ptr<drake::automotive::Curve2<double>> curve)
      : curve_(std::move(curve)) {}

  /// Returns the curve the car will follow.
  const drake::automotive::Curve2<double>* get_raw_curve() const {
    return curve_.get();
  }

 private:
  std::unique_ptr<drake::automotive::Curve2<double>> curve_;
};

}  // namespace delphyne
