// Copyright 2018 Toyota Research Institute

#include "backend/simulation_run_stats.h"

#include "gtest/gtest.h"

#include "backend/delphyne_realtime_clock.h"
#include "backend/delphyne_time_point.h"

namespace delphyne {
namespace backend {

const double kTimeTolerance{1e-8};

GTEST_TEST(SimulationRunStatsTest, UsualRunTest) {
  const double sim_start = 1.0;
  const TimePoint realtime_start = RealtimeClock::now();

  SimulationRunStats stats = SimulationRunStats(1.0, realtime_start);

  EXPECT_EQ(0, stats.get_executed_steps());
  EXPECT_EQ(sim_start, stats.get_start_simtime());
  EXPECT_EQ(realtime_start, stats.get_start_realtime());
  EXPECT_NEAR(0., stats.ElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0., stats.ElapsedRealtime(), kTimeTolerance);

  // Run one step
  double step_simtime = sim_start + 0.1;
  TimePoint step_realtime = realtime_start + std::chrono::milliseconds(100);

  stats.StepExecuted(step_simtime, step_realtime);

  EXPECT_EQ(1, stats.get_executed_steps());
  EXPECT_EQ(sim_start, stats.get_start_simtime());
  EXPECT_EQ(realtime_start, stats.get_start_realtime());
  EXPECT_EQ(step_simtime, stats.get_last_step_simtime());
  EXPECT_EQ(step_realtime, stats.get_last_step_realtime());
  EXPECT_NEAR(0.1, stats.ElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0.1, stats.ElapsedRealtime(), kTimeTolerance);

  // Run a second step
  step_simtime = sim_start + 0.2;
  step_realtime = realtime_start + std::chrono::milliseconds(500);

  stats.StepExecuted(step_simtime, step_realtime);

  EXPECT_EQ(2, stats.get_executed_steps());
  EXPECT_EQ(sim_start, stats.get_start_simtime());
  EXPECT_EQ(realtime_start, stats.get_start_realtime());
  EXPECT_EQ(step_simtime, stats.get_last_step_simtime());
  EXPECT_EQ(step_realtime, stats.get_last_step_realtime());
  EXPECT_NEAR(0.2, stats.ElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0.5, stats.ElapsedRealtime(), kTimeTolerance);
}

GTEST_TEST(SimulationRunStatsTest, CantChangeAfterRunIsDoneTest) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  const double sim_start = 1.0;
  const TimePoint realtime_start = RealtimeClock::now();

  SimulationRunStats stats = SimulationRunStats(1.0, realtime_start);

  // Run one step.
  double step_simtime = sim_start + 0.1;
  TimePoint step_realtime = realtime_start + std::chrono::milliseconds(100);

  stats.StepExecuted(step_simtime, step_realtime);

  // Mark the simulation run as done.
  stats.RunFinished();

  // Attempting to run another step should fail.
  step_simtime = sim_start + 0.2;
  step_realtime = realtime_start + std::chrono::milliseconds(200);

  EXPECT_DEATH(stats.StepExecuted(step_simtime, step_realtime),
               "condition '!run_finished_' failed.");
}

}  // namespace backend
}  // namespace delphyne
