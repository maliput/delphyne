// Copyright 2022 Toyota Research Institute.
#include "backend/fixed_phase_iteration_handler.h"

#include <sstream>

#include <ignition/common/Console.hh>
#include <maliput/base/manual_phase_provider.h>

#include "delphyne/macros.h"

namespace delphyne {

using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;

namespace {

// Returns a string containing the serialization of @p current_phases in the form:
// " {phase_ring_id_a: phase_a, phase_ring_id_b: phase_b, ..., phase_ring_id_x: phase_x}
std::string PhasesToString(const std::vector<std::pair<PhaseRing::Id, Phase::Id>>& current_phases) {
  std::stringstream ss;
  ss << "{ ";
  for (size_t i = 0; i < current_phases.size(); ++i) {
    if (i != 0) {
      ss << ", ";
    }
    ss << current_phases[i].first.string() << ": " << current_phases[i].second.string();
  }
  ss << " }";
  return ss.str();
}

}  // namespace

FixedPhaseIterationHandler::FixedPhaseIterationHandler(maliput::api::RoadNetwork* road_network, double phase_duration)
    : DynamicEnvironmentHandler(road_network), phase_duration_(phase_duration) {
  DELPHYNE_VALIDATE(phase_duration > 0., std::invalid_argument, "Phase duration should be greater than zero.");

  // Advertise a service for modifying the phase duration.
  if (!node_.Advertise(kSetPhaseDurationSrvName, &FixedPhaseIterationHandler::SetPhaseDurationSvCb, this)) {
    ignerr << "Error advertising service [" << kSetPhaseDurationSrvName << "]"
           << "\n Phase duration won't be able to be modified" << std::endl;
  }

  // Create publisher for publishing the phases.
  pub_ = node_.Advertise<ignition::msgs::StringMsg>(kCurrentPhaseTopic);
  if (!pub_) {
    ignerr << "Error advertising topic [" << kCurrentPhaseTopic << "]" << std::endl;
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

  std::vector<std::pair<PhaseRing::Id, Phase::Id>> current_phases;
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
    current_phases.emplace_back(phase_ring_id, new_phase_id);
  }
  // Publish current phase.
  ignition::msgs::StringMsg msg;
  msg.set_data(PhasesToString(current_phases));
  const bool res = pub_.Publish(msg);
  if (!res) {
    ignerr << "Error publishing info about phase rings" << std::endl;
  }
}

void FixedPhaseIterationHandler::set_phase_duration(double phase_duration) {
  DELPHYNE_VALIDATE(phase_duration > 0., std::invalid_argument, "Phase duration should be greater than zero.");
  phase_duration_ = phase_duration;
}

double FixedPhaseIterationHandler::get_phase_duration() const { return phase_duration_; }

}  // namespace delphyne
