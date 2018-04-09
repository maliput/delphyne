// Copyright 2018 Toyota Research Institute

#include "backend/simulation_run_stats.h"
#include "backend/delphyne_realtime_clock.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

SimulationRunStats::SimulationRunStats(double start_simtime)
    : SimulationRunStats(start_simtime, RealtimeClock::now()) {}

SimulationRunStats::SimulationRunStats(double start_simtime,
                                       const TimePoint& start_realtime)
    : start_simtime_(start_simtime), start_realtime_(start_realtime) {}

void SimulationRunStats::StepExecuted(double simtime) {
  StepExecuted(simtime, RealtimeClock::now());
}

void SimulationRunStats::StepExecuted(double simtime,
                                      const TimePoint& realtime) {
  DELPHYNE_ASSERT(!run_finished_);
  executed_steps_++;
  last_step_simtime_ = simtime;
  last_step_realtime_ = realtime;
}

double SimulationRunStats::ElapsedSimtime() const {
  if (executed_steps_ == 0) {
    return 0.0;
  }
  return last_step_simtime_ - start_simtime_;
}

double SimulationRunStats::ElapsedRealtime() const {
  if (executed_steps_ == 0) {
    return 0.0;
  }
  const Duration delta_realtime = last_step_realtime_ - start_realtime_;
  auto secs =
      std::chrono::duration_cast<std::chrono::duration<double>>(delta_realtime);
  return secs.count();
}

void SimulationRunStats::RunFinished() { run_finished_ = true; }

}  // namespace backend
}  // namespace delphyne
