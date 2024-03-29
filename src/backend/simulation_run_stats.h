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

#pragma once

#include <chrono>
#include <limits>

// public headers
#include "delphyne/types.h"

// private headers
#include "delphyne/macros.h"

namespace delphyne {

/// @brief A class that keeps statistics of a single simulation run.
/// An interactive session is usually composed of one or more simulation runs,
/// depending on the interactions with the external world. Each simulation run
/// captures the moment it started (both in simulation and real time), the
/// number of simulation steps executed, the timestamps for that last step
/// (again, in simulation and real time) and a flag indicating if the run has
/// finished (once this happens no more steps can be recorded).
class SimulationRunStats {
 public:
  /// @brief Creates a new simulation run, starting at `start_simtime`. Assumes
  /// that the real-time start is the current time.
  ///
  /// @param[in] start_simtime The time the simulation started, in seconds,
  /// given by the simulator clock.
  ///
  /// @param[in] expected_realtime_rate The desired real time based in the
  /// simulation runner configuration.
  explicit SimulationRunStats(double start_simtime, double expected_realtime_rate);

  /// @brief Creates a new simulation run.
  ///
  /// @param[in] start_simtime The time the simulation started, in seconds,
  /// given by the simulator clock.
  ///
  /// @param[in] expected_realtime_rate The desired real time based in the
  /// simulation runner configuration.
  ///
  /// @param[in] start_realtime The time the simulation started, given by the
  /// wall clock.
  explicit SimulationRunStats(double start_simtime, double expected_realtime_rate, const TimePoint& start_realtime);

  /// @brief Records that a step was executed by the simulator.
  ///
  /// @param[in] simtime The time the step was executed, in seconds, given by
  /// the simulator clock.
  void StepExecuted(double simtime);

  /// @brief Records the current real time, typically used after a
  /// simulation step has occurred
  ///
  /// @param[in] realtime The time the step was executed, given by the
  /// wall clock.
  void SetRealtime(const TimePoint& realtime);

  /// @brief Returns the simulation time difference, in seconds, since the
  /// run started until the last step.
  double ElapsedSimtime() const;

  /// @brief Returns the real time difference, in seconds, since the run
  /// started until the last step.
  double ElapsedRealtime() const;

  /// @brief Indicates that the run has finished.
  void RunFinished();

  /// @brief Returns the actual real time rate based on elapsed simulation and
  /// real time.
  ///
  /// @pre executed_steps_ > 0
  double EffectiveRealtimeRate();

  /// @brief Returns the number of executed steps.
  int get_executed_steps() const { return executed_steps_; }

  /// @brief Returns the start time, based on simulator clock.
  double get_start_simtime() const { return start_simtime_; }

  /// @brief Returns the simulation time of the last recorded step.
  double get_last_step_simtime() const { return last_step_simtime_; }

  /// @brief Returns the start time, based on real time clock.
  const TimePoint& get_start_realtime() const { return start_realtime_; }

  /// @brief Returns the real time of the last recorded step.
  const TimePoint& get_last_step_realtime() const { return last_step_realtime_; }

  double get_expected_realtime_rate() const { return expected_realtime_rate_; }

 private:
  // @brief Start time, in seconds, based on simulator clock.
  double start_simtime_{0.0};

  /// @brief Last recorded step in simulation time. Measured in seconds.
  double last_step_simtime_{0.0};

  // @brief The configured real time rate at which the simulation should run.
  double expected_realtime_rate_{1.0};

  // @brief Start time, in real-time.
  TimePoint start_realtime_;

  // @brief Last recorded step in real-time.
  TimePoint last_step_realtime_;

  // @brief Number of executed steps.
  int executed_steps_{0};

  // @brief Flag stating if the run has finished.
  bool run_finished_{false};
};

}  // namespace delphyne
