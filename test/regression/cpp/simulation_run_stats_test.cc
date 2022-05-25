// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "backend/simulation_run_stats.h"

#include <gtest/gtest.h>

#include "delphyne/types.h"

namespace delphyne {

const double kTimeTolerance{1e-8};
GTEST_TEST(SimulationRunStatsTest, UsualRunTest) {
  const double sim_start = 1.0;
  const double realtime_rate = 1.1;
  const TimePoint realtime_start = RealtimeClock::now();

  SimulationRunStats stats(sim_start, realtime_rate, realtime_start);

  EXPECT_EQ(0, stats.get_executed_steps());
  EXPECT_EQ(sim_start, stats.get_start_simtime());
  EXPECT_EQ(realtime_rate, stats.get_expected_realtime_rate());
  EXPECT_EQ(realtime_start, stats.get_start_realtime());
  EXPECT_NEAR(0., stats.ElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0., stats.ElapsedRealtime(), kTimeTolerance);

  // Run one step
  double step_simtime = sim_start + 0.1;
  TimePoint step_realtime = realtime_start + std::chrono::milliseconds(100);

  stats.StepExecuted(step_simtime);
  stats.SetRealtime(step_realtime);

  EXPECT_EQ(1, stats.get_executed_steps());
  EXPECT_EQ(sim_start, stats.get_start_simtime());
  EXPECT_EQ(realtime_start, stats.get_start_realtime());
  EXPECT_EQ(step_simtime, stats.get_last_step_simtime());
  EXPECT_EQ(step_realtime, stats.get_last_step_realtime());
  EXPECT_NEAR(0.1, stats.ElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0.1, stats.ElapsedRealtime(), kTimeTolerance);
  EXPECT_NEAR(1.0, stats.EffectiveRealtimeRate(), kTimeTolerance);

  // Run a second step
  step_simtime = sim_start + 0.2;
  step_realtime = realtime_start + std::chrono::milliseconds(400);

  stats.StepExecuted(step_simtime);
  stats.SetRealtime(step_realtime);

  EXPECT_EQ(2, stats.get_executed_steps());
  EXPECT_EQ(sim_start, stats.get_start_simtime());
  EXPECT_EQ(realtime_start, stats.get_start_realtime());
  EXPECT_EQ(step_simtime, stats.get_last_step_simtime());
  EXPECT_EQ(step_realtime, stats.get_last_step_realtime());
  EXPECT_NEAR(0.2, stats.ElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0.4, stats.ElapsedRealtime(), kTimeTolerance);
  EXPECT_NEAR(0.5, stats.EffectiveRealtimeRate(), kTimeTolerance);
}
GTEST_TEST(SimulationRunStatsTest, CantChangeAfterRunIsDoneTest) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  const double sim_start = 1.0;
  const double realtime_rate = 1.1;
  const TimePoint realtime_start = RealtimeClock::now();

  SimulationRunStats stats(sim_start, realtime_rate, realtime_start);

  // Run one step.
  double step_simtime = sim_start + 0.1;
  TimePoint step_realtime = realtime_start + std::chrono::milliseconds(100);

  stats.StepExecuted(step_simtime);
  stats.SetRealtime(step_realtime);

  // Mark the simulation run as done.
  stats.RunFinished();

  // Attempting to run another step should fail.
  step_simtime = sim_start + 0.2;
  step_realtime = realtime_start + std::chrono::milliseconds(200);

#ifndef NDEBUG
  // Death tests are explicitly disabled when not compiling for debug
  // because the assertions these expect are not armed (i.e. the macros
  // expand to nothing) in this case.
  EXPECT_DEATH(stats.StepExecuted(step_simtime), "condition '!run_finished_' failed.");
  EXPECT_DEATH(stats.SetRealtime(step_realtime), "condition '!run_finished_' failed.");
#endif
}

}  // namespace delphyne
