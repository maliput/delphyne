// Copyright 2018 Toyota Research Institute

#include "backend/interactive_simulation_stats.h"

#include "gtest/gtest.h"

#include "backend/delphyne_realtime_clock.h"
#include "backend/delphyne_time_point.h"

namespace delphyne {
namespace backend {

const double kTimeTolerance{1e-8};

GTEST_TEST(InteractiveSimulationStatsTest, UsualRunTest) {
  double current_run_simtime_start;
  TimePoint current_run_realtime_start;

  InteractiveSimulationStats stats;

  // Nothing has been yet simulated.
  EXPECT_EQ(0, stats.TotalRuns());
  EXPECT_EQ(0, stats.TotalExecutedSteps());
  EXPECT_NEAR(0., stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0., stats.TotalElapsedRealtime(), kTimeTolerance);

  // A new simulation run has started, but no step recorded.
  current_run_simtime_start = 0.0;
  current_run_realtime_start = RealtimeClock::now();

  stats.NewRunStartingAt(current_run_simtime_start, current_run_realtime_start);

  EXPECT_EQ(1, stats.TotalRuns());
  EXPECT_EQ(0, stats.TotalExecutedSteps());
  EXPECT_NEAR(0., stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0., stats.TotalElapsedRealtime(), kTimeTolerance);

  // Execute a step
  stats.GetMutableCurrentRunStats()->StepExecuted(
      current_run_simtime_start + 0.1,
      current_run_realtime_start + std::chrono::milliseconds(200));

  EXPECT_EQ(1, stats.TotalRuns());
  EXPECT_EQ(1, stats.TotalExecutedSteps());
  EXPECT_NEAR(0.1, stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0.2, stats.TotalElapsedRealtime(), kTimeTolerance);

  // Execute another step of the current run and add another run with a single
  // step that started 10 seconds after.
  stats.GetMutableCurrentRunStats()->StepExecuted(
      current_run_simtime_start + 0.3,
      current_run_realtime_start + std::chrono::milliseconds(500));

  current_run_simtime_start = 0.01;
  current_run_realtime_start =
      current_run_realtime_start + std::chrono::seconds(10);

  stats.NewRunStartingAt(current_run_simtime_start, current_run_realtime_start);
  stats.GetMutableCurrentRunStats()->StepExecuted(
      current_run_simtime_start + 1.1,
      current_run_realtime_start + std::chrono::milliseconds(800));

  EXPECT_EQ(2, stats.TotalRuns());
  EXPECT_EQ(3, stats.TotalExecutedSteps());
  EXPECT_NEAR(1.4, stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(1.3, stats.TotalElapsedRealtime(), kTimeTolerance);
}

}  // namespace backend
}  // namespace delphyne
