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

#include "backend/interactive_simulation_stats.h"

#include <cmath>

#include <gtest/gtest.h>

#include "delphyne/types.h"

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

  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate, current_run_realtime_start);

  EXPECT_EQ(1, stats.TotalRuns());
  EXPECT_EQ(0, stats.TotalExecutedSteps());
  EXPECT_NEAR(0., stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0., stats.TotalElapsedRealtime(), kTimeTolerance);

  // Execute a step
  stats.StepExecuted(current_run_simtime_start + 0.1);
  stats.RealtimeStepExecuted(current_run_realtime_start + std::chrono::milliseconds(200));

  EXPECT_EQ(1, stats.TotalRuns());
  EXPECT_EQ(1, stats.TotalExecutedSteps());
  EXPECT_NEAR(0.1, stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(0.2, stats.TotalElapsedRealtime(), kTimeTolerance);

  // Execute another step of the current run and add another run with a single
  // step that started 10 seconds after.
  stats.StepExecuted(current_run_simtime_start + 0.3);
  stats.RealtimeStepExecuted(current_run_realtime_start + std::chrono::milliseconds(500));

  current_run_simtime_start = 0.01;
  current_run_realtime_start = current_run_realtime_start + std::chrono::seconds(10);

  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate, current_run_realtime_start);
  stats.StepExecuted(current_run_simtime_start + 1.1);
  stats.RealtimeStepExecuted(current_run_realtime_start + std::chrono::milliseconds(800));

  EXPECT_EQ(2, stats.TotalRuns());
  EXPECT_EQ(3, stats.TotalExecutedSteps());
  EXPECT_NEAR(1.4, stats.TotalElapsedSimtime(), kTimeTolerance);
  EXPECT_NEAR(1.3, stats.TotalElapsedRealtime(), kTimeTolerance);
}
GTEST_TEST(InteractiveSimulationStatsTest, RealtimeComputation) {
  const double realtime_rate = 1.0;

  InteractiveSimulationStats stats;

  // Nothing has been yet simulated.
  EXPECT_TRUE(std::isnan(stats.get_current_realtime_rate()));

  double current_run_simtime_start = 0.0;
  TimePoint current_run_realtime_start = RealtimeClock::now();

  // First run at 1.0 real-time rate
  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate, current_run_realtime_start);

  for (int i = 1; i < 10; i++) {
    current_run_simtime_start += 0.1;
    current_run_realtime_start += std::chrono::milliseconds(100);
    stats.StepExecuted(current_run_simtime_start);
    stats.RealtimeStepExecuted(current_run_realtime_start);
  }

  EXPECT_EQ(realtime_rate, stats.GetCurrentRunStats().get_expected_realtime_rate());
  EXPECT_NEAR(1.0, stats.get_current_realtime_rate(), kTimeTolerance);

  // Second run, even if configure at 1.0 real-time rate, it will have an
  // effective rate of 0.1 (as if the simulator was lagging behind). Note
  // also that there is a gap in time between the first and second run, like
  // if the interactive simulation was paused for some time
  current_run_simtime_start += 0.1;
  current_run_realtime_start += std::chrono::seconds(50);

  stats.NewRunStartingAt(current_run_simtime_start, realtime_rate, current_run_realtime_start);

  // In 350 simulation steps we should have got asymptotically near to the
  // effective real-time rate (0.1).
  for (int i = 1; i < 350; i++) {
    current_run_simtime_start += 0.01;
    current_run_realtime_start += std::chrono::milliseconds(100);
    stats.StepExecuted(current_run_simtime_start);
    stats.RealtimeStepExecuted(current_run_realtime_start);
  }

  EXPECT_EQ(realtime_rate, stats.GetCurrentRunStats().get_expected_realtime_rate());
  EXPECT_NEAR(0.1, stats.get_current_realtime_rate(), kTimeTolerance);
}

}  // namespace delphyne
