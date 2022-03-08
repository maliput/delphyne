// Copyright 2022 Toyota Research Institute.
#pragma once

#include <maliput/api/road_network.h>
#include <maliput/common/maliput_copyable.h>

#include "delphyne/macros.h"

namespace delphyne {

/// Abstract API for managing the rules dynamic states of a maliput::api::RoadNetwork.
/// The states are expected to change based on time.
class DynamicEnvironmentHandler {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(DynamicEnvironmentHandler)
  DynamicEnvironmentHandler() = delete;

  virtual ~DynamicEnvironmentHandler() = default;

  /// Updates the rule's states.
  /// @param sim_time Current simulation time.
  virtual void Update(double sim_time) = 0;

 protected:
  /// Creates DynamicEnvironmentHandler
  /// @param road_network maliput::api::RoadNetwork pointer.
  DynamicEnvironmentHandler(maliput::api::RoadNetwork* road_network) : road_network_(road_network) {
    DELPHYNE_VALIDATE(road_network_ != nullptr, std::invalid_argument, "RoadNetwork can't be nullptr.");
  }

  maliput::api::RoadNetwork* road_network_{nullptr};
};

}  // namespace delphyne
