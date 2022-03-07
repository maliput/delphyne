// Copyright 2022 Toyota Research Institute.
#include "backend/fixed_phase_iteration_handler.h"

#include <ignition/common/Console.hh>
#include <maliput/base/manual_phase_provider.h>

#include "delphyne/macros.h"

namespace delphyne {

FixedPhaseIterationHandler::FixedPhaseIterationHandler(maliput::api::RoadNetwork* road_network, double phase_duration)
    : DynamicEnvironmentHandler(road_network), phase_duration_(phase_duration) {
  DELPHYNE_VALIDATE(phase_duration > 0., std::invalid_argument, "Phase duration should be greater than zero.");

  // Advertise a service for modifying the phase duration.
  if (!node_.Advertise(kSetPhaseDurationSrvName, &FixedPhaseIterationHandler::SetPhaseDurationSvCb, this)) {
    ignerr << "Error advertising service [" << kSetPhaseDurationSrvName << "]"
           << "\n Phase duration won't be able to be modified" << std::endl;
  }
}

void FixedPhaseIterationHandler::SetPhaseDurationSvCb(const ignition::msgs::Double& phase_duration) {
  if (phase_duration.data() <= 0.) {
    ignerr << "srv: " << kSetPhaseDurationSrvName << " -- Phase duration should be greater than zero." << std::endl;
    return;
  }
  set_phase_duration(phase_duration.data());
}

void FixedPhaseIterationHandler::Update(double sim_time) {
  if (!(sim_time - last_sim_time_ > phase_duration_)) {
    return;
  }
  last_sim_time_ = sim_time;

  auto phase_provider = dynamic_cast<maliput::ManualPhaseProvider*>(road_network_->phase_provider());
  const auto phase_ring_book = road_network_->phase_ring_book();
  for (const auto& phase_ring_id : phase_ring_book->GetPhaseRings()) {
    const auto phase_ring = phase_ring_book->GetPhaseRing(phase_ring_id);
    const auto phase_provider_result = phase_provider->GetPhase(phase_ring_id);
    DELPHYNE_VALIDATE(phase_provider_result != std::nullopt, std::runtime_error,
                      "Missing current phase for PhaseRingId: " + phase_ring_id.string());

    if (!phase_provider_result->next.has_value()) {
      continue;
    }
    const auto new_phase_id = phase_provider_result->next.value().state;
    const auto next_phases = phase_ring->GetNextPhases(new_phase_id);
    phase_provider->SetPhase(phase_ring_id, new_phase_id, next_phases.front().id, next_phases.front().duration_until);
  }
}

void FixedPhaseIterationHandler::set_phase_duration(double phase_duration) {
  DELPHYNE_VALIDATE(phase_duration > 0., std::invalid_argument, "Phase duration should be greater than zero.");
  phase_duration_ = phase_duration;
}

double FixedPhaseIterationHandler::get_phase_duration() const { return phase_duration_; }

}  // namespace delphyne
