// Copyright 2022 Toyota Research Institute.
#pragma once

#include <atomic>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <maliput/api/road_network.h>
#include <maliput/common/maliput_copyable.h>

#include "backend/dynamic_environment_handler.h"
#include "delphyne/macros.h"

namespace delphyne {

/// DynamicEnvironmentHandler class implementation.
/// Each rule state is expected to last a fixed amount of time.
/// An ignition service is provided for modifying the phase duration.
class FixedPhaseIterationHandler : public DynamicEnvironmentHandler {
 public:
  /// Name of service for modifying phase duration.
  static constexpr char kSetPhaseDurationSrvName[] = "/set_phase_duration";

  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedPhaseIterationHandler)
  FixedPhaseIterationHandler() = delete;

  /// Constructs a FixedPhaseIterationHandler.
  /// @param road_network maliput::api::RoadNetwork pointer.
  /// @param phase_duration The duration of the rule's states in seconds.
  FixedPhaseIterationHandler(maliput::api::RoadNetwork* road_network, double phase_duration = 10.);

  ~FixedPhaseIterationHandler() override = default;

  void Update(double sim_time) override;

  /// @returns The duration of the rule's states in seconds.
  double get_phase_duration() const;

  /// @param phase_duration The duration of the rule's states in seconds.
  void set_phase_duration(double phase_duration);

 protected:
  void SetPhaseDurationSvCb(const ignition::msgs::Double& phase_duration);

 private:
  std::atomic<double> phase_duration_{10.};
  ignition::transport::Node node_;
  double last_sim_time_{0};
};

}  // namespace delphyne
