// Copyright 2017 Toyota Research Institute

#include <memory>
#include <utility>

#include "drake/automotive/curve2.h"

#include "../../include/delphyne/agent_plugin_base.h"

namespace delphyne {

class TrajectoryCarParams final : public delphyne::AgentPluginParams {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryCarParams)

  explicit TrajectoryCarParams(
      std::unique_ptr<drake::automotive::Curve2<double>> curve)
      : curve_(std::move(curve)) {}

  drake::automotive::Curve2<double>* get_raw_curve() { return curve_.get(); }

 private:
  std::unique_ptr<drake::automotive::Curve2<double>> curve_;
};

}  // namespace delphyne
