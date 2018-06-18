// Copyright 2018 Toyota Research Institute

#include "backend/interactive_simulation_stats.h"

namespace delphyne {

void InteractiveSimulationStats::NewRunStartingAt(
    double start_simtime, double expected_realtime_rate) {
  NewRunStartingAt(start_simtime, expected_realtime_rate, RealtimeClock::now());
}

void InteractiveSimulationStats::NewRunStartingAt(
    double start_simtime, double expected_realtime_rate,
    const TimePoint& start_realtime) {
  std::lock_guard<std::mutex> lock(mutex_);
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
  run_stats_.push_back(SimulationRunStats(start_simtime, expected_realtime_rate,
                                          start_realtime));
}

const SimulationRunStats& InteractiveSimulationStats::GetCurrentRunStats()
    const {
  std::lock_guard<std::mutex> lock(mutex_);
  DELPHYNE_DEMAND(!run_stats_.empty());
  return run_stats_.at(run_stats_.size() - 1);
}

SimulationRunStats* InteractiveSimulationStats::GetMutableCurrentRunStats() {
  DELPHYNE_DEMAND(!run_stats_.empty());
  return &run_stats_.back();
}

void InteractiveSimulationStats::StepExecuted(double simtime) {
  StepExecuted(simtime, RealtimeClock::now());
}

void InteractiveSimulationStats::StepExecuted(double simtime,
                                              const TimePoint& realtime) {
  UpdateWeightedRealtimeRate(simtime, realtime);
  GetMutableCurrentRunStats()->StepExecuted(simtime, realtime);
}

const TimePoint InteractiveSimulationStats::CurrentStepExpectedRealtimeEnd()
    const {
  auto current_run = GetCurrentRunStats();
  const double current_realtime_rate = current_run.get_expected_realtime_rate();
  const double current_elapsed_simtime = current_run.ElapsedSimtime();
  return current_run.get_start_realtime() +
         Duration(current_elapsed_simtime / current_realtime_rate);
}

void InteractiveSimulationStats::UpdateWeightedRealtimeRate(
    double simtime, const TimePoint& realtime) {
  // Control how much weight are we giving to the previous steps. A low value
  // (i.e. towards 0) will make the real-time rate very sensitive to current
  // changes but also very unstable. A high value (i.e. towards 1.0) would make
  // the real-time rate more stable and take longer to catch-up with recent
  // changes.
  static const double kWeighFactor{0.95};

  auto current_run = GetCurrentRunStats();

  const double simtime_passed = simtime - current_run.get_last_step_simtime();
  const Duration realtime_passed =
      realtime - current_run.get_last_step_realtime();

  weighted_simtime_ = weighted_simtime_ * kWeighFactor + simtime_passed;
  weighted_realtime_ =
      weighted_realtime_ * kWeighFactor + realtime_passed.count();

  weighted_realtime_rate_ = weighted_simtime_ / weighted_realtime_;
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

}  // namespace delphyne
