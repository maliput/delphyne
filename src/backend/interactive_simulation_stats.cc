// Copyright 2018 Toyota Research Institute

#include "backend/interactive_simulation_stats.h"

namespace delphyne {

static constexpr double kWeighFactor{0.95};

void InteractiveSimulationStats::NewRunStartingAt(double start_simtime, double expected_realtime_rate) {
  NewRunStartingAt(start_simtime, expected_realtime_rate, RealtimeClock::now());
}

void InteractiveSimulationStats::NewRunStartingAt(double start_simtime, double expected_realtime_rate,
                                                  const TimePoint& start_realtime) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!run_stats_.empty()) {
    SimulationRunStats* current = GetUnsafeMutableCurrentRunStats();
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
  run_stats_.push_back(SimulationRunStats(start_simtime, expected_realtime_rate, start_realtime));
}

SimulationRunStats InteractiveSimulationStats::GetCurrentRunStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return GetUnsafeCurrentRunStats();
}

const SimulationRunStats& InteractiveSimulationStats::GetUnsafeCurrentRunStats() const {
  DELPHYNE_VALIDATE(!run_stats_.empty(), std::runtime_error, "Runtime statistics are empty");
  return run_stats_.at(run_stats_.size() - 1);
}

SimulationRunStats* InteractiveSimulationStats::GetUnsafeMutableCurrentRunStats() {
  DELPHYNE_VALIDATE(!run_stats_.empty(), std::runtime_error, "Runtime statistics are empty");
  return &run_stats_.back();
}

void InteractiveSimulationStats::RealtimeStepExecuted() { RealtimeStepExecuted(RealtimeClock::now()); }

TimePoint InteractiveSimulationStats::StepExecuted(double simtime) {
  std::lock_guard<std::mutex> lock(mutex_);
  UpdateWeightedSimtimeRate(simtime);
  GetUnsafeMutableCurrentRunStats()->StepExecuted(simtime);
  return UnsafeCurrentStepExpectedRealtimeEnd();
}

void InteractiveSimulationStats::RealtimeStepExecuted(const TimePoint& realtime) {
  std::lock_guard<std::mutex> lock(mutex_);
  UpdateWeightedRealtimeRate(realtime);
  GetUnsafeMutableCurrentRunStats()->SetRealtime(realtime);
}

const TimePoint InteractiveSimulationStats::UnsafeCurrentStepExpectedRealtimeEnd() const {
  const SimulationRunStats& current_run = GetUnsafeCurrentRunStats();

  const double current_realtime_rate = current_run.get_expected_realtime_rate();
  const double current_elapsed_simtime = current_run.ElapsedSimtime();
  return current_run.get_start_realtime() + Duration(current_elapsed_simtime / current_realtime_rate);
}

void InteractiveSimulationStats::UpdateWeightedRealtimeRate(const TimePoint& realtime) {
  // Control how much weight are we giving to the previous steps. A low value
  // (i.e. towards 0) will make the real-time rate very sensitive to current
  // changes but also very unstable. A high value (i.e. towards 1.0) would make
  // the real-time rate more stable and take longer to catch-up with recent
  // changes.
  const SimulationRunStats& current_run = GetUnsafeCurrentRunStats();

  const Duration realtime_passed = realtime - current_run.get_last_step_realtime();

  weighted_realtime_ = weighted_realtime_ * kWeighFactor + realtime_passed.count();

  weighted_realtime_rate_ = weighted_simtime_ / weighted_realtime_;
}

void InteractiveSimulationStats::UpdateWeightedSimtimeRate(double simtime) {
  // Control how much weight are we giving to the previous steps. A low value
  // (i.e. towards 0) will make the real-time rate very sensitive to current
  // changes but also very unstable. A high value (i.e. towards 1.0) would make
  // the real-time rate more stable and take longer to catch-up with recent
  // changes.
  const SimulationRunStats& current_run = GetUnsafeCurrentRunStats();

  const double simtime_passed = simtime - current_run.get_last_step_simtime();

  weighted_simtime_ = weighted_simtime_ * kWeighFactor + simtime_passed;
}

double InteractiveSimulationStats::TotalElapsedSimtime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (run_stats_.empty()) {
    return 0.0;
  }
  return GetUnsafeCurrentRunStats().ElapsedSimtime() + total_elapsed_simtime_;
}

double InteractiveSimulationStats::TotalElapsedRealtime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (run_stats_.empty()) {
    return 0.0;
  }
  return GetUnsafeCurrentRunStats().ElapsedRealtime() + total_elapsed_realtime_;
}

int InteractiveSimulationStats::TotalExecutedSteps() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (run_stats_.empty()) {
    return 0;
  }
  return GetUnsafeCurrentRunStats().get_executed_steps() + total_executed_steps_;
}

int InteractiveSimulationStats::TotalRuns() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return run_stats_.size();
}

}  // namespace delphyne
