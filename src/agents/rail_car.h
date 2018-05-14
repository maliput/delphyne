// Copyright 2017 Toyota Research Institute

#include <memory>
#include <utility>

#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/lane_direction.h"

#include "../../include/delphyne/agent_plugin_base.h"

namespace delphyne {

class RailCarParams final : public delphyne::AgentPluginParams {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RailCarParams)

  RailCarParams(
      std::unique_ptr<drake::automotive::LaneDirection> lane_direction,
      std::unique_ptr<drake::automotive::MaliputRailcarParams<double>>
          start_params)
      : lane_direction_(std::move(lane_direction)),
        start_params_(std::move(start_params)) {}

  drake::automotive::LaneDirection* get_raw_lane_direction() {
    return lane_direction_.get();
  }

  drake::automotive::MaliputRailcarParams<double>* get_raw_start_params() {
    return start_params_.get();
  }

 private:
  std::unique_ptr<drake::automotive::LaneDirection> lane_direction_;

  std::unique_ptr<drake::automotive::MaliputRailcarParams<double>>
      start_params_;
};

}  // namespace delphyne
