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

// private headers
#include "delphyne/macros.h"

namespace delphyne {

SimulationRunStats::SimulationRunStats(double start_simtime, double expected_realtime_rate)
    : SimulationRunStats(start_simtime, expected_realtime_rate, RealtimeClock::now()) {}

SimulationRunStats::SimulationRunStats(double start_simtime, double expected_realtime_rate,
                                       const TimePoint& start_realtime)
    : start_simtime_(start_simtime),
      last_step_simtime_(start_simtime),
      expected_realtime_rate_(expected_realtime_rate),
      start_realtime_(start_realtime),
      last_step_realtime_(start_realtime) {}

void SimulationRunStats::StepExecuted(double simtime) {
  DELPHYNE_ASSERT(!run_finished_);
  executed_steps_++;
  last_step_simtime_ = simtime;
}

void SimulationRunStats::SetRealtime(const TimePoint& realtime) {
  DELPHYNE_ASSERT(!run_finished_);
  last_step_realtime_ = realtime;
}

double SimulationRunStats::ElapsedSimtime() const { return last_step_simtime_ - start_simtime_; }

double SimulationRunStats::ElapsedRealtime() const {
  const Duration delta_realtime = last_step_realtime_ - start_realtime_;
  auto secs = std::chrono::duration_cast<std::chrono::duration<double>>(delta_realtime);
  return secs.count();
}

void SimulationRunStats::RunFinished() { run_finished_ = true; }

double SimulationRunStats::EffectiveRealtimeRate() {
  DELPHYNE_ASSERT(executed_steps_ > 0);
  return ElapsedSimtime() / ElapsedRealtime();
}

}  // namespace delphyne
