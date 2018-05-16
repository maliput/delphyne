// Copyright 2017 Toyota Research Institute

#include <memory>
#include <utility>

#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/lane_direction.h"

#include "include/delphyne/agent_plugin_base.h"

namespace delphyne {

/// This class models the required extra parameters to create a railcar.
class RailCarAgentParams final : public delphyne::AgentPluginParams {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RailCarAgentParams)

  /// Default constructor.
  ///
  /// @param[in] initial_lane_direction The initial lane direction of travel.
  ///
  /// @param[in] start_params The parameters that the car will use as a start
  /// state. See MaliputRailcarParams in Drake.
  RailCarAgentParams(
      std::unique_ptr<drake::automotive::LaneDirection> lane_direction,
      std::unique_ptr<drake::automotive::MaliputRailcarParams<double>>
          start_params)
      : lane_direction_(std::move(lane_direction)),
        start_params_(std::move(start_params)) {}

  /// Returns the initial lane travel direction.
  const drake::automotive::LaneDirection* get_raw_lane_direction() const {
    return lane_direction_.get();
  }

  /// Returns the initial car start parameters.
  const drake::automotive::MaliputRailcarParams<double>* get_raw_start_params()
      const {
    return start_params_.get();
  }

 private:
  std::unique_ptr<drake::automotive::LaneDirection> lane_direction_;

  std::unique_ptr<drake::automotive::MaliputRailcarParams<double>>
      start_params_;
};

}  // namespace delphyne
