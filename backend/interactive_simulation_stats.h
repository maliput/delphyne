// Copyright 2018 Toyota Research Institute

#pragma once

#include <limits>
#include <vector>

#include "backend/simulation_run_stats.h"
#include "backend/system.h"

namespace delphyne {

/// @brief A class that keeps statistics on a interactive simulation session
/// An interactive session is usually composed of one or more simulation runs,
/// depending on the interactions with the external world. As an example
/// consider the following scenario, where the simulation step is of 1ms:
/// - An interactive simulation starts at a 1.0 real-time rate and is paused
///   after 5 secs.
/// - Some time T1 goes by.
/// - The real-time rate is changed to 2.0 and a request to execute 2000 steps
///   is placed, each step taking 1ms sim time.
/// - Some time T2 goes by.
/// - The real-time rate is changed to 0.5 and the simulation is unpaused.
/// - The simulation is stopped after 10 secs.
///
/// For this case, the interactive simulation statistics would be composed of
/// three runs:
/// - A 5 sec (both simulation time and real time) run, with 5000 steps.
/// - A 2 sec simulation time, 1 sec real time run, with 2000 steps.
/// - A 5 sec simulation time, 10 sec real time run, with 5000 steps.
///
/// Hence the total numbers would be:
/// - 12 secs sim time
/// - 16 secs real time
/// - 12000 steps
///
/// Note that T1 and T2 are not considered, as during those time the simulation
/// was idle.
class InteractiveSimulationStats {
 public:
  /// @brief Creates a new simulation run, starting at `start_simtime`
  ///
  /// @param[in] start_simtime. The time the simulation started, given by the
  /// simulator clock.
  ///
  /// @param[in] expected_realtime_rate The desired real time based in the
  /// simulation runner configuration.
  void NewRunStartingAt(double start_simtime, double expected_realtime_rate);

  /// @brief Creates a new simulation run, starting at `start_simtime`
  ///
  /// @param[in] start_simtime. The time the simulation started, given by the
  /// simulator clock.
  ///
  /// @param[in] expected_realtime_rate The desired real time based in the
  /// simulation runner configuration.
  ///
  /// @param[in] start_realtime. The time the simulation started, given by the
  /// real-time clock.
  void NewRunStartingAt(double start_simtime, double expected_realtime_rate,
                        const TimePoint& start_realtime);

  /// @brief Records that a step was executed by the simulator and records it as
  /// part of the current simulation run. Assumes that the realtime counterpart
  /// is the current time.
  ///
  /// @param[in] simtime The time the step was executed, in seconds, given by
  /// the simulator clock.
  void StepExecuted(double simtime);

  /// @brief Records that a step was executed by the simulator and records it as
  /// part of the current simulation run.
  ///
  /// @param[in] simtime The time the step was executed, in seconds, given by
  /// the simulator clock.
  ///
  /// @param[in] realtime The time the step was executed, given by the
  /// wall clock.
  void StepExecuted(double simtime, const TimePoint& realtime);

  /// @brief Returns the current running simulation stats @see
  /// SimulationRunStats
  const SimulationRunStats& GetCurrentRunStats() const;

  /// @brief Returns the sum of the elapsed simulation time of all the
  /// simulation runs.
  double TotalElapsedSimtime() const;

  /// @brief Returns the sum of the elapsed real time of all the simulation
  /// runs.
  double TotalElapsedRealtime() const;

  /// @brief Returns the sum of the executed steps of all the simulation runs.
  int TotalExecutedSteps() const;

  /// @brief Returns the number of simulation runs.
  int TotalRuns() const;

  /// @brief Returns a TimePoint representing the real-time value at which
  /// the current simulation step is supposed to end. To compute this value
  /// (which should theoretically be in the future) it considers the simulation
  /// time that took the current step and the configured real-time rate.
  /// Finally, note that if the simulation is too slow to keep up with the
  /// real-time factor, the returned object will be in the past and not in the
  /// future.
  const TimePoint CurrentStepExpectedRealtimeEnd() const;

  /// @brief Returns the current real-time rate by doing a weighted cumulative
  /// sum. We use this method instead of just dividing `total_elapsed_simtime_`
  /// by `total_elapsed_simtime_` because that would require a long time to
  /// adjust to real-time rate changes.
  /// As an example of this behavior consider an interactive simulation
  /// configured with a 1ms simulation step. Assuming that it starts with a
  /// simulation run of 100 seconds at 1.0 real-time, ff we then change the
  /// real-time rate to 0.1, after the first execution step we would have:
  /// - Total elapsed simulation time: 100s + 0.001s = 100.001s
  /// - Total elapsed real time: 100s + 0.01s = 100.01s
  /// whose real-time rate would yield 0.99991. If we take this further and let
  /// the second run execute for 100s (i.e. 10,000 steps) we would get:
  /// - Total elapsed simulation time: 110s
  /// - Total elapsed real time: 200s
  /// whose real-time rate would yield 0.55, which is still far away from the
  /// 0.1 configured step.
  ///
  /// In contrast, by using a weighted cumulative sum to compute the real-time
  /// rate, the latest steps are given more importance in the computation,
  /// allowing the rate to quickly update to configuration changes. Continuing
  /// with the above example and using a 0.5 weighing factor, in only 15 steps
  /// the real-time rate drops to ~0.102, closely matching our 0.1 configured
  /// factor.
  double get_current_realtime_rate() const { return weighted_realtime_rate_; }

 private:
  // Returns the current running simulation stats @see SimulationRunStats
  SimulationRunStats* GetMutableCurrentRunStats();

  // Updates the value of the `weighted_realtime_rate_` field based on the
  // elapsed simulation and real time of an executed step.
  void UpdateWeightedRealtimeRate(double simtime, const TimePoint& realtime);

  // @brief All the recorded simulation runs
  std::vector<SimulationRunStats> run_stats_;

  // @brief Cache the total elapsed simulation time.
  double total_elapsed_simtime_{0.};

  // @brief Cache the total elapsed real time.
  double total_elapsed_realtime_{0.};

  // @brief Cache the total executed steps.
  int total_executed_steps_{0};

  // @brief The weighed accumulated simulation time, since the start of the
  // interactive simulation. Note that the time the simulation is paused is
  // not considered.
  double weighted_simtime_{0.};

  // @brief The weighed accumulated real time, since the start of the
  // interactive simulation. Note that the time the simulation is paused is
  // not considered.
  double weighted_realtime_{0.};

  // @brief The ratio between weighed simulation time and weighed real time.
  double weighted_realtime_rate_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace delphyne
