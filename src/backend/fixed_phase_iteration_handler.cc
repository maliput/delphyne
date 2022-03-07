// Copyright 2022 Toyota Research Institute.
#include "backend/fixed_phase_iteration_handler.h"

#include <maliput/base/manual_phase_provider.h>

#include "delphyne/macros.h"

namespace delphyne {

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

}  // namespace delphyne