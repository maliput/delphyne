// Copyright 2018 Toyota Research Institute

#include <math.h>

#include "backend/interactive_simulation_stats.h"

#include "gtest/gtest.h"

#include "backend/delphyne_realtime_clock.h"
#include "backend/delphyne_time_point.h"

namespace delphyne {

const double kTimeTolerance{1e-8};

GTEST_TEST(InteractiveSimulationStatsTest, UsualRunTest) {
  const double realtime_rate = 1.1;

  InteractiveSimulationStats stats;

  // Nothing has been yet simulated.
  EXPECT_EQ(0, stats.TotalRuns());
  EXPECT_EQ(0, stats.TotalExecutedSteps());
  EXPECT_NEAR(0., stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0., stats.TotalElapsedRealtime(), kTimeTolerance);

  // A new simulation run has started, but no step recorded.
  double current_run_simtime_start = 0.0;
  TimePoint current_run_realtime_start = RealtimeClock::now();

  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate,
                         current_run_realtime_start);

  EXPECT_EQ(1, stats.TotalRuns());
  EXPECT_EQ(0, stats.TotalExecutedSteps());
  EXPECT_NEAR(0., stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0., stats.TotalElapsedRealtime(), kTimeTolerance);

  // Execute a step
  stats.StepExecuted(
      current_run_simtime_start + 0.1,
      current_run_realtime_start + std::chrono::milliseconds(200));

  EXPECT_EQ(1, stats.TotalRuns());
  EXPECT_EQ(1, stats.TotalExecutedSteps());
  EXPECT_NEAR(0.1, stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0.2, stats.TotalElapsedRealtime(), kTimeTolerance);

  // Execute another step of the current run and add another run with a single
  // step that started 10 seconds after.
  stats.StepExecuted(
      current_run_simtime_start + 0.3,
      current_run_realtime_start + std::chrono::milliseconds(500));

  current_run_simtime_start = 0.01;
  current_run_realtime_start =
      current_run_realtime_start + std::chrono::seconds(10);

  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate,
                         current_run_realtime_start);
  stats.StepExecuted(
      current_run_simtime_start + 1.1,
      current_run_realtime_start + std::chrono::milliseconds(800));

  EXPECT_EQ(2, stats.TotalRuns());
  EXPECT_EQ(3, stats.TotalExecutedSteps());
  EXPECT_NEAR(1.4, stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(1.3, stats.TotalElapsedRealtime(), kTimeTolerance);
}

GTEST_TEST(InteractiveSimulationStatsTest, RealtimeComputation) {
  const double realtime_rate = 1.0;

  InteractiveSimulationStats stats;

  // Nothing has been yet simulated.
  EXPECT_TRUE(isnan(stats.get_current_realtime_rate()));

  double current_run_simtime_start = 0.0;
  TimePoint current_run_realtime_start = RealtimeClock::now();

  // First run at 1.0 real-time rate
  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate,
                         current_run_realtime_start);

  for (int i = 1; i < 10; i++) {
    current_run_simtime_start += 0.1;
    current_run_realtime_start += std::chrono::milliseconds(100);
    stats.StepExecuted(current_run_simtime_start, current_run_realtime_start);
  }

  EXPECT_EQ(realtime_rate,
            stats.GetCurrentRunStats().get_expected_realtime_rate());
  EXPECT_NEAR(1.0, stats.get_current_realtime_rate(), kTimeTolerance);

  // Second run, even if configure at 1.0 real-time rate, it will have an
  // effective rate of 0.1 (as if the simulator was lagging behind). Note
  // also that there is a gap in time between the first and second run, like
  // if the interactive simulation was paused for some time
  current_run_simtime_start += 0.1;
  current_run_realtime_start += std::chrono::seconds(50);

  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate,
                         current_run_realtime_start);

  // In 350 simulation steps we should have got asymptotically near to the
  // effective real-time rate (0.1).
  for (int i = 1; i < 350; i++) {
    current_run_simtime_start += 0.01;
    current_run_realtime_start += std::chrono::milliseconds(100);
    stats.StepExecuted(current_run_simtime_start, current_run_realtime_start);
  }

  EXPECT_EQ(realtime_rate,
            stats.GetCurrentRunStats().get_expected_realtime_rate());
  EXPECT_NEAR(0.1, stats.get_current_realtime_rate(), kTimeTolerance);
}

}  // namespace delphyne
