/**
 * @file python/delphyne/simulation.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// public headers
#include "delphyne/mi6/agent_simulation.h"

// private headers
#include "backend/agent_simulation_builder.h"
#include "backend/interactive_simulation_stats.h"
#include "backend/simulation_run_stats.h"
#include "backend/simulation_runner.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

using delphyne::AgentBlueprint;
using delphyne::AgentCollision;
using delphyne::AgentSimulation;
using delphyne::AgentSimulationBuilder;
using delphyne::InteractiveSimulationStats;
using delphyne::SimulationRunner;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(simulation, m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("maliput.api");

  py::class_<InteractiveSimulationStats>(m, "InteractiveSimulationStats")
      .def(py::init<>())
      .def("total_elapsed_simtime", &InteractiveSimulationStats::TotalElapsedSimtime)
      .def("total_elapsed_realtime", &InteractiveSimulationStats::TotalElapsedRealtime)
      .def("total_executed_steps", &InteractiveSimulationStats::TotalExecutedSteps)
      .def("total_runs", &InteractiveSimulationStats::TotalRuns)
      .def("get_current_realtime_rate", &InteractiveSimulationStats::get_current_realtime_rate);

  py::class_<SimulationRunner>(m, "SimulationRunner")
      .def(py::init<std::unique_ptr<AgentSimulation>, double>(),
           "Load the simulation and initialise it to run"
           "at the specified time step.",
           py::arg("simulation"), py::arg("time_step"))
      .def(py::init<std::unique_ptr<AgentSimulation>, double, bool, bool>(),
           "Load the simulation and initialise it to run"
           "at the specified time step and whether you wish"
           "the simulation to start paused and with logging enabled.",
           py::arg("simulation"), py::arg("time_step"), py::arg("paused"), py::arg("log"))
      .def(py::init<std::unique_ptr<AgentSimulation>, double, bool, bool, std::string>(),
           "Load the simulation and initialise it to run"
           "at the specified time step and whether you wish"
           "the simulation to start paused and with logging enabled,"
           "allowing you to also set a custom logfile name.",
           py::arg("simulation"), py::arg("time_step"), py::arg("paused"), py::arg("log"), py::arg("logfile_name"))
      .def(py::init<std::unique_ptr<AgentSimulation>, double, double>(),
           "Load the simulation and initialise it to run"
           "at the specified time step and realtime rate.",
           py::arg("simulation"), py::arg("time_step"), py::arg("realtime_rate"))
      .def(py::init<std::unique_ptr<AgentSimulation>, double, double, bool, bool, std::string>(),
           "Load the simulation and initialise time step, realtime rate"
           "and whether you wish the simulation to start paused and with "
           "logging enabled.",
           py::arg("simulation"), py::arg("time_step"), py::arg("realtime_rate"), py::arg("paused"), py::arg("log"),
           py::arg("logfile_name"))
      .def("set_realtime_rate", &SimulationRunner::SetRealtimeRate)
      .def("get_realtime_rate", &SimulationRunner::GetRealtimeRate)
      .def("start", &SimulationRunner::Start)
      .def("stop", &SimulationRunner::Stop)
      .def("run_async_for", &SimulationRunner::RunAsyncFor)
      .def("run_sync_for", &SimulationRunner::RunSyncFor)
      .def("is_interactive_loop_running", &SimulationRunner::IsInteractiveLoopRunning)
      .def("add_step_callback", &SimulationRunner::AddStepCallback)
      .def("add_collision_callback", &SimulationRunner::AddCollisionCallback)
      .def("enable_collisions", &SimulationRunner::EnableCollisions)
      .def("disable_collisions", &SimulationRunner::DisableCollisions)
      .def("is_simulation_paused", &SimulationRunner::IsSimulationPaused)
      .def("pause_simulation", &SimulationRunner::PauseSimulation)
      .def("unpause_simulation", &SimulationRunner::UnpauseSimulation)
      .def("request_simulation_step_execution", &SimulationRunner::RequestSimulationStepExecution)
      .def("get_simulation", &SimulationRunner::GetSimulation, py::return_value_policy::reference_internal)
      .def("get_mutable_simulation", &SimulationRunner::GetMutableSimulation,
           py::return_value_policy::reference_internal)
      .def("get_stats", &SimulationRunner::GetStats, py::return_value_policy::reference)
      .def("is_logging", &SimulationRunner::IsLogging)
      .def("start_logging", (void (SimulationRunner::*)(void)) & SimulationRunner::StartLogging)
      .def("start_logging", (void (SimulationRunner::*)(const std::string&)) & SimulationRunner::StartLogging)
      .def("stop_logging", &SimulationRunner::StopLogging)
      .def("get_log_filename", &SimulationRunner::GetLogFilename);

  py::class_<AgentCollision>(m, "AgentCollision")
      .def_readonly("agents", &AgentCollision::agents)
      .def_readonly("location", &AgentCollision::location);

  py::class_<AgentSimulationBuilder>(m, "AgentSimulationBuilder")
      .def(py::init([](void) { return std::make_unique<AgentSimulationBuilder>(); }))
      .def("add_agent",
           [](AgentSimulationBuilder* self, std::unique_ptr<AgentBlueprint> blueprint) {
             return self->AddAgent(std::move(blueprint));
           },
           py::return_value_policy::reference_internal)
      .def("set_road_geometry",
           py::overload_cast<std::unique_ptr<const ::maliput::api::RoadGeometry>>(
               &AgentSimulationBuilder::SetRoadGeometry),
           py::return_value_policy::reference_internal, "Sets road geometry for the simulation to be built",
           py::arg("road_geometry"))
      .def("set_road_geometry",
           py::overload_cast<std::unique_ptr<const ::maliput::api::RoadGeometry>,
                             const ::maliput::utility::ObjFeatures&>(&AgentSimulationBuilder::SetRoadGeometry),
           py::return_value_policy::reference_internal, "Sets road geometry for the simulation to be built",
           py::arg("road_geometry"), py::arg("features"))
      .def("get_road_geometry", &AgentSimulationBuilder::GetRoadGeometry,
           py::return_value_policy::reference_internal)
      .def("set_road_network",
           py::overload_cast<std::unique_ptr<const ::maliput::api::RoadNetwork>>(
               &AgentSimulationBuilder::SetRoadNetwork),
           py::return_value_policy::reference_internal, "Sets road network for the simulation to be built",
           py::arg("road_network"))
      .def(
          "set_road_network",
          py::overload_cast<std::unique_ptr<const ::maliput::api::RoadNetwork>, const ::maliput::utility::ObjFeatures&>(
              &AgentSimulationBuilder::SetRoadNetwork),
          py::return_value_policy::reference_internal, "Sets road network for the simulation to be built",
          py::arg("road_network"), py::arg("features"))
      .def("build", &AgentSimulationBuilder::Build);

  py::class_<AgentSimulation>(m, "AgentSimulation")
      .def("get_collisions", &AgentSimulation::GetCollisions, py::return_value_policy::reference_internal)
      .def("get_agent_by_name",
           [](AgentSimulation* self, const std::string& name) { return &self->GetAgentByName(name); },
           py::return_value_policy::reference_internal)
      .def("get_mutable_agent_by_name",
           [](AgentSimulation* self, const std::string& name) { return self->GetMutableAgentByName(name); },
           py::return_value_policy::reference_internal)
      .def("get_diagram", &AgentSimulation::GetDiagram, py::return_value_policy::reference_internal)
      .def("get_context", &AgentSimulation::GetContext, py::return_value_policy::reference_internal)
      .def("get_mutable_context", &AgentSimulation::GetMutableContext, py::return_value_policy::reference_internal)
      .def("get_current_time", &AgentSimulation::GetCurrentTime);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
