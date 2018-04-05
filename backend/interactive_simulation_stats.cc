// Copyright 2018 Toyota Research Institute

#include "backend/interactive_simulation_stats.h"

namespace delphyne {
namespace backend {

void InteractiveSimulationStats::NewRunStartingAt(double start_simtime) {
  NewRunStartingAt(start_simtime, RealtimeClock::now());
}

void InteractiveSimulationStats::NewRunStartingAt(
    double start_simtime, const TimePoint& start_realtime) {
  if (!run_stats_.empty()) {
    SimulationRunStats* current = GetMutableCurrentRunStats();
    current->RunFinished();
    // Discard an empty run
    if (current->get_executed_steps() == 0) {
      run_stats_.pop_back();
    } else {
      total_elapsed_simtime_ += current->ElapsedSimtime();
      total_elapsed_realtime_ += current->ElapsedRealtime();
      total_executed_steps_ += current->get_executed_steps();
    }
  }
  run_stats_.push_back(SimulationRunStats(start_simtime, start_realtime));
}

const SimulationRunStats& InteractiveSimulationStats::GetCurrentRunStats()
    const {
  DELPHYNE_DEMAND(!run_stats_.empty());
  return run_stats_.at(run_stats_.size() - 1);
}

SimulationRunStats* InteractiveSimulationStats::GetMutableCurrentRunStats() {
  DELPHYNE_DEMAND(!run_stats_.empty());
  return &run_stats_.back();
}

double InteractiveSimulationStats::TotalElapsedSimtime() const {
  if (run_stats_.empty()) {
    return 0.0;
  }
  return GetCurrentRunStats().ElapsedSimtime() + total_elapsed_simtime_;
}

double InteractiveSimulationStats::TotalElapsedRealtime() const {
  if (run_stats_.empty()) {
    return 0.0;
  }
  return GetCurrentRunStats().ElapsedRealtime() + total_elapsed_realtime_;
}

int InteractiveSimulationStats::TotalExecutedSteps() const {
  if (run_stats_.empty()) {
    return 0;
  }
  return GetCurrentRunStats().get_executed_steps() + total_executed_steps_;
}

int InteractiveSimulationStats::TotalRuns() const { return run_stats_.size(); }

}  // namespace backend
}  // namespace delphyne
