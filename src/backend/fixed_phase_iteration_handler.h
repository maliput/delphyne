// Copyright 2022 Toyota Research Institute.
#pragma once

#include <maliput/api/road_network.h>
#include <maliput/common/maliput_copyable.h>

#include "backend/dynamic_environment_handler.h"
#include "delphyne/macros.h"

namespace delphyne {

/// DynamicEnvironmentHandler class implementation.
/// Each rule state is expected to last a fixed amount of time.
class FixedPhaseIterationHandler : public DynamicEnvironmentHandler {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedPhaseIterationHandler)
  FixedPhaseIterationHandler() = delete;

  /// Constructs a FixedPhaseIterationHandler.
  /// @param road_network maliput::api::RoadNetwork pointer.
  /// @param phase_duration The duration of the rule's states in seconds.
  FixedPhaseIterationHandler(maliput::api::RoadNetwork* road_network, double phase_duration = 10.)
      : DynamicEnvironmentHandler(road_network), phase_duration_(phase_duration) {
    DELPHYNE_VALIDATE(phase_duration > 0., std::invalid_argument, "Phase duration should be greater than zero.");
  }

  ~FixedPhaseIterationHandler() override = default;

  void Update(double sim_time) override;

  /// @returns The duration of the rule's states in seconds.
  double get_phase_duration() const { return phase_duration_; }

  /// @param phase_duration The duration of the rule's states in seconds.
  void set_phase_duration(double phase_duration) { phase_duration_ = phase_duration; }

 private:
  double phase_duration_{10};
  double last_sim_time_{0};
};

}  // namespace delphyne
