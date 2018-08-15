/**
 * @file python/delphyne/simulation.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// private headers
#include "backend/automotive_simulator.h"
#include "backend/interactive_simulation_stats.h"
#include "backend/simulation_run_stats.h"
#include "backend/simulation_runner.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

using delphyne::AutomotiveSimulator;
using delphyne::SimulatorRunner;
using delphyne::InteractiveSimulationStats;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(simulation, m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.maliput.api");

  py::class_<InteractiveSimulationStats>(m, "InteractiveSimulationStats")
      .def(py::init<>())
      .def("total_elapsed_simtime",
           &InteractiveSimulationStats::TotalElapsedSimtime)
      .def("total_elapsed_realtime",
           &InteractiveSimulationStats::TotalElapsedRealtime)
      .def("total_executed_steps",
           &InteractiveSimulationStats::TotalExecutedSteps)
      .def("total_runs", &InteractiveSimulationStats::TotalRuns)
      .def("get_current_realtime_rate",
           &InteractiveSimulationStats::get_current_realtime_rate);

  py::class_<SimulatorRunner>(m, "SimulatorRunner")
      .def(py::init<std::unique_ptr<AutomotiveSimulator<double>>, double>(),
           "Load the simulator and initialise it to run"
           "at the specified time step.",
           py::arg("simulator"), py::arg("time_step"))
      .def(py::init<std::unique_ptr<AutomotiveSimulator<double>>, double, bool,
                    bool>(),
           "Load the simulator and initialise it to run"
           "at the specified time step and whether you wish"
           "the simulation to start paused and with logging enabled.",
           py::arg("simulator"), py::arg("time_step"), py::arg("paused"),
           py::arg("log"))
      .def(py::init<std::unique_ptr<AutomotiveSimulator<double>>, double, bool,
                    bool, std::string>(),
           "Load the simulator and initialise it to run"
           "at the specified time step and whether you wish"
           "the simulation to start paused and with logging enabled,"
           "allowing you to also set a custom logfile name.",
           py::arg("simulator"), py::arg("time_step"), py::arg("paused"),
           py::arg("log"), py::arg("logfile_name"))
      .def(py::init<std::unique_ptr<AutomotiveSimulator<double>>, double,
                    double>(),
           "Load the simulator and initialise it to run"
           "at the specified time step and realtime rate.",
           py::arg("simulator"), py::arg("time_step"), py::arg("realtime_rate"))
      .def(py::init<std::unique_ptr<AutomotiveSimulator<double>>, double,
                    double, bool, bool, std::string>(),
           "Load the simulator and initialise time step, realtime rate"
           "and whether you wish the simulation to start paused and with "
           "logging enabled.",
           py::arg("simulator"), py::arg("time_step"), py::arg("realtime_rate"),
           py::arg("paused"), py::arg("log"), py::arg("logfile_name"))
      .def("set_realtime_rate", &SimulatorRunner::SetRealtimeRate)
      .def("get_realtime_rate", &SimulatorRunner::GetRealtimeRate)
      .def("start", &SimulatorRunner::Start)
      .def("stop", &SimulatorRunner::Stop)
      .def("run_async_for", &SimulatorRunner::RunAsyncFor)
      .def("run_sync_for", &SimulatorRunner::RunSyncFor)
      .def("is_interactive_loop_running",
           &SimulatorRunner::IsInteractiveLoopRunning)
      .def("add_step_callback", &SimulatorRunner::AddStepCallback)
      .def("add_collision_callback", &SimulatorRunner::AddCollisionCallback)
      .def("enable_collisions", &SimulatorRunner::EnableCollisions)
      .def("disable_collisions", &SimulatorRunner::DisableCollisions)
      .def("is_simulation_paused", &SimulatorRunner::IsSimulationPaused)
      .def("pause_simulation", &SimulatorRunner::PauseSimulation)
      .def("unpause_simulation", &SimulatorRunner::UnpauseSimulation)
      .def("request_simulation_step_execution",
           &SimulatorRunner::RequestSimulationStepExecution)
      .def("get_simulator",
           &SimulatorRunner::GetSimulator,
           py::return_value_policy::reference_internal)
      .def("get_mutable_simulator",
           &SimulatorRunner::GetMutableSimulator,
           py::return_value_policy::reference_internal)
      .def("get_stats", &SimulatorRunner::get_stats,
           py::return_value_policy::reference)
      .def("is_logging",
           &SimulatorRunner::IsLogging)
      .def("start_logging",
           (void (SimulatorRunner::*)(void))&SimulatorRunner::StartLogging)
      .def("start_logging",
          (void (SimulatorRunner::*)(const std::string&))
          &SimulatorRunner::StartLogging)
      .def("stop_logging",
           &SimulatorRunner::StopLogging)
      .def("get_log_filename",
           &SimulatorRunner::GetLogFilename);

  py::class_<AutomotiveSimulator<double>>(m, "AutomotiveSimulator")
      .def(py::init(
          [](void) { return std::make_unique<AutomotiveSimulator<double>>(); }))
      .def("add_agent", &AutomotiveSimulator<double>::AddAgent,
           py::return_value_policy::reference_internal)
      .def("get_collisions", &AutomotiveSimulator<double>::GetCollisions,
           py::return_value_policy::reference_internal)
      .def("start", &AutomotiveSimulator<double>::Start)
      .def("set_road_geometry", &AutomotiveSimulator<double>::SetRoadGeometry,
           "Transfer a road geometry to the control of the simulator",
           py::arg("road_geometry"))
      .def("get_current_simulation_time",
           &AutomotiveSimulator<double>::GetCurrentSimulationTime)
      .def("get_mutable_context",
           &AutomotiveSimulator<double>::GetMutableContext,
           py::return_value_policy::reference_internal);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
