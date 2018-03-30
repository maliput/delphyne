// Copyright 2018 Toyota Research Institute

#pragma once

#include <chrono>
#include <limits>

using Clock = std::chrono::steady_clock;
using Duration = std::chrono::duration<double>;
using TimePoint = std::chrono::time_point<Clock, Duration>;

namespace delphyne {
namespace backend {

/// @brief A class that keeps statistics of a single simulation run.
/// An interactive session is usually composed of one or more simulation runs,
/// depending on the interactions with the external world. Each simulation run
/// captures the moment it started (both in simulation and real time), the
/// number of simulation steps executed, the timestamps for that last step
/// (again, in simulation and real time) and a flag indicating if the run has
/// finished (once this happens no more steps can be recorded).
class SimulationRunStats {
 public:
  /// @brief Creates a new simulation run, starting at start_simtime. Assumes
  /// that the `start_realtime` is the current time.
  ///
  /// @param[in] start_simtime. The time the simulation started, given by the
  /// simulator clock.
  explicit SimulationRunStats(double start_simtime);

  /// @brief Creates a new simulation run, starting at start_simtime.
  ///
  /// @param[in] start_simtime. The time the simulation started, given by the
  /// simulator clock.
  ///
  /// @param[in] start_realtime. The time the simulation started, given by the
  /// wall clock.
  explicit SimulationRunStats(double start_simtime,
                              const TimePoint& start_realtime);

  /// @brief Records that a step was executed by the simulator. Assumes that
  /// the realtime counterpart is the current time.
  ///
  /// @param[in] simtime. The time the step was executed, given by the
  /// simulator clock.
  void StepExecuted(double simtime);

  /// @brief Records that a step was executed by the simulator.
  ///
  /// @param[in] simtime. The time the step was executed, given by the
  /// simulator clock.
  ///
  /// @param[in] realtime. The time the step was executed, given by the
  /// wall clock.
  void StepExecuted(double simtime, const TimePoint& realtime);

  /// @brief Returns the simulation time difference, in seconds, since the
  /// run started until the last step.
  double ElapsedSimtime();

  /// @brief Returns the real time difference, in seconds, since the run
  /// started until the last step.
  double ElapsedRealtime();

  /// @brief Indicates that the run has finished.
  void RunFinished();

  /// @brief Returns the number of executed steps.
  int get_executed_steps() { return executed_steps_; }

  /// @brief Returns the start time, based on simulator clock.
  double get_start_simtime() { return start_simtime_; }

  /// @brief Returns the simulation time of the last recorded step.
  double get_last_step_simtime() { return last_step_simtime_; }

  /// @brief Returns the start time, based on in wall clock.
  TimePoint get_start_realtime() { return start_realtime_; }

  /// @brief Returns the real time of the last recorded step.
  TimePoint get_last_step_realtime() { return last_step_realtime_; }

 private:
  // @brief Start time, based on simulator clock.
  double start_simtime_;

  /// @brief Last recorded step in simulation time.
  double last_step_simtime_{std::numeric_limits<double>::quiet_NaN()};

  // @brief Start time, based on wall clock.
  TimePoint start_realtime_;

  // @brief Last recorded step in real time.
  TimePoint last_step_realtime_;

  // @brief Number of executed steps.
  int executed_steps_{0};

  // @brief Flag stating if the run has finished.
  bool run_finished_{false};
};

}  // namespace backend
}  // namespace delphyne
