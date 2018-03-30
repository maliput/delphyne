// Copyright 2018 Toyota Research Institute

#pragma once

#include <memory>
#include <vector>

#include "backend/simulation_run_stats.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

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
  /// @brief Creates a new simulation run, starting at start_simtime
  ///
  /// @param[in] start_simtime. The time the simulation started, given by the
  /// simulator clock.
  void NewRunStartingAt(double start_simtime,
                        TimePoint start_realtime = Clock::now());

  /// @brief Returns the current running simulation stats @see
  /// SimulationRunStats
  SimulationRunStats* GetCurrentRunStats();

  /// @brief Returns the sum of the elapsed simulation time of all the
  /// simulation runs.
  double TotalElapsedSimtime();

  /// @brief Returns the sum of the elapsed real time of all the simulation
  /// runs.
  double TotalElapsedRealtime();

  /// @brief Returns the sum of the executed steps of all the simulation runs.
  int TotalExecutedSteps();

  /// @brief Returns the number of simulation runs.
  int TotalRuns();

 private:
  // @brief All the recorded simulation runs
  std::vector<SimulationRunStats> run_stats_;

  // @brief Cache the total elapsed simulation time.
  double total_elapsed_simtime_{0.};

  // @brief Cache the total elapsed real time.
  double total_elapsed_realtime_{0.};

  // @brief Cache the total executed steps.
  int total_executed_steps_{0};
};

}  // namespace backend
}  // namespace delphyne
