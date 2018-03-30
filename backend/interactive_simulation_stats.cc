// Copyright 2018 Toyota Research Institute

#include "backend/interactive_simulation_stats.h"

namespace delphyne {
namespace backend {

void InteractiveSimulationStats::NewRunStartingAt(double start_simtime,
                                                  TimePoint start_realtime) {
  if (!run_stats_.empty()) {
    auto current = GetCurrentRunStats();
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

SimulationRunStats* InteractiveSimulationStats::GetCurrentRunStats() {
  DELPHYNE_DEMAND(!run_stats_.empty());
  return &run_stats_.back();
}

double InteractiveSimulationStats::TotalElapsedSimtime() {
  if (run_stats_.empty()) {
    return 0.0;
  }
  return GetCurrentRunStats()->ElapsedSimtime() + total_elapsed_simtime_;
}

double InteractiveSimulationStats::TotalElapsedRealtime() {
  if (run_stats_.empty()) {
    return 0.0;
  }
  return GetCurrentRunStats()->ElapsedRealtime() + total_elapsed_realtime_;
}

int InteractiveSimulationStats::TotalExecutedSteps() {
  if (run_stats_.empty()) {
    return 0;
  }
  return GetCurrentRunStats()->get_executed_steps() + total_executed_steps_;
}

int InteractiveSimulationStats::TotalRuns() { return run_stats_.size(); }

}  // namespace backend
}  // namespace delphyne
