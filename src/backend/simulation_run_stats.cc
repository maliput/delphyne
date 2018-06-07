// Copyright 2018 Toyota Research Institute

#include "backend/simulation_run_stats.h"

#include "backend/delphyne_realtime_clock.h"
#include "backend/system.h"

namespace delphyne {

SimulationRunStats::SimulationRunStats(double start_simtime,
                                       double expected_realtime_rate)
    : SimulationRunStats(start_simtime, expected_realtime_rate,
                         RealtimeClock::now()) {}

SimulationRunStats::SimulationRunStats(double start_simtime,
                                       double expected_realtime_rate,
                                       const TimePoint& start_realtime)
    : start_simtime_(start_simtime),
      last_step_simtime_(start_simtime),
      expected_realtime_rate_(expected_realtime_rate),
      start_realtime_(start_realtime),
      last_step_realtime_(start_realtime) {}

void SimulationRunStats::StepExecuted(double simtime,
                                      const TimePoint& realtime) {
  DELPHYNE_ASSERT(!run_finished_);
  executed_steps_++;
  last_step_simtime_ = simtime;
  last_step_realtime_ = realtime;
}

double SimulationRunStats::ElapsedSimtime() const {
  return last_step_simtime_ - start_simtime_;
}

double SimulationRunStats::ElapsedRealtime() const {
  const Duration delta_realtime = last_step_realtime_ - start_realtime_;
  auto secs =
      std::chrono::duration_cast<std::chrono::duration<double>>(delta_realtime);
  return secs.count();
}

void SimulationRunStats::RunFinished() { run_finished_ = true; }

double SimulationRunStats::EffectiveRealtimeRate() {
  DELPHYNE_ASSERT(executed_steps_ > 0);
  return ElapsedSimtime() / ElapsedRealtime();
}

}  // namespace delphyne
