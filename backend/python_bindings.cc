// Copyright 2017 Toyota Research Institute

#include <memory>
#include <vector>

#include <drake/automotive/gen/maliput_railcar_params.h>
#include <drake/common/find_resource.h>
#include <drake/systems/framework/basic_vector.h>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "backend/automotive_simulator.h"
#include "backend/interactive_simulation_stats.h"
#include "backend/road_builder.h"
#include "backend/simulation_run_stats.h"
#include "backend/simulation_runner.h"

#include "include/delphyne/agent_plugin_base.h"
#include "src/agents/mobil_car.h"
#include "src/agents/rail_car.h"
#include "src/agents/trajectory_car.h"

namespace py = pybind11;

using std::unique_ptr;

using delphyne::AutomotiveSimulator;
using delphyne::RoadBuilder;
using delphyne::SimulatorRunner;
using delphyne::InteractiveSimulationStats;
using delphyne::AgentPluginParams;
using delphyne::RailCarAgentParams;
using delphyne::TrajectoryCarAgentParams;
using delphyne::MobilCarAgentParams;
using drake::automotive::LaneDirection;
using drake::automotive::MaliputRailcarParams;
using drake::automotive::MaliputRailcarState;
using drake::maliput::api::RoadGeometry;
using drake::systems::BasicVector;

namespace {
PYBIND11_MODULE(python_bindings, m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.maliput.api");

  py::class_<AgentPluginParams>(m, "AgentPluginParams");

  py::class_<RailCarAgentParams, AgentPluginParams>(m, "RailCarAgentParams")
      .def(py::init<
           std::unique_ptr<drake::automotive::LaneDirection>,
           std::unique_ptr<drake::automotive::MaliputRailcarParams<double>>>());

  py::class_<TrajectoryCarAgentParams, AgentPluginParams>(
      m, "TrajectoryCarAgentParams")
      .def(py::init<std::unique_ptr<drake::automotive::Curve2<double>>>());

  py::class_<MobilCarAgentParams, AgentPluginParams>(m, "MobilCarAgentParams")
      .def(py::init<bool>());

  py::class_<MaliputRailcarState<double>, BasicVector<double>>(
      m, "MaliputRailcarState")
      .def(py::init<>())
      .def_property("s", &MaliputRailcarState<double>::s,
                    &MaliputRailcarState<double>::set_s)
      .def_property("speed", &MaliputRailcarState<double>::speed,
                    &MaliputRailcarState<double>::set_speed);

  py::class_<MaliputRailcarParams<double>, BasicVector<double>>(
      m, "MaliputRailcarParams")
      .def(py::init<>())
      .def_property("r", &MaliputRailcarParams<double>::r,
                    &MaliputRailcarParams<double>::set_r)
      .def_property("h", &MaliputRailcarParams<double>::h,
                    &MaliputRailcarParams<double>::set_h)
      .def_property("max_speed", &MaliputRailcarParams<double>::max_speed,
                    &MaliputRailcarParams<double>::set_max_speed)
      .def_property("velocity_limit_kp",
                    &MaliputRailcarParams<double>::velocity_limit_kp,
                    &MaliputRailcarParams<double>::set_velocity_limit_kp);

  py::class_<InteractiveSimulationStats>(m, "InteractiveSimulationStats")
      .def(py::init<>())
      .def("TotalElapsedSimtime",
           &InteractiveSimulationStats::TotalElapsedSimtime)
      .def("TotalElapsedRealtime",
           &InteractiveSimulationStats::TotalElapsedRealtime)
      .def("TotalExecutedSteps",
           &InteractiveSimulationStats::TotalExecutedSteps)
      .def("TotalRuns", &InteractiveSimulationStats::TotalRuns)
      .def("get_current_realtime_rate",
           &InteractiveSimulationStats::get_current_realtime_rate);

  py::class_<SimulatorRunner>(m, "SimulatorRunner")
      .def(py::init<unique_ptr<AutomotiveSimulator<double>>, double>())
      .def(py::init<unique_ptr<AutomotiveSimulator<double>>, double, bool>())
      .def(py::init<unique_ptr<AutomotiveSimulator<double>>, double, double>())
      .def(py::init<unique_ptr<AutomotiveSimulator<double>>, double, double,
                    bool>())
      .def("SetRealtimeRate", &SimulatorRunner::SetRealtimeRate)
      .def("GetRealtimeRate", &SimulatorRunner::GetRealtimeRate)
      .def("Start", &SimulatorRunner::Start)
      .def("Stop", &SimulatorRunner::Stop)
      .def("RunAsyncFor", &SimulatorRunner::RunAsyncFor)
      .def("RunSyncFor", &SimulatorRunner::RunSyncFor)
      .def("IsInteractiveLoopRunning",
           &SimulatorRunner::IsInteractiveLoopRunning)
      .def("AddStepCallback", &SimulatorRunner::AddStepCallback)
      .def("IsSimulationPaused", &SimulatorRunner::IsSimulationPaused)
      .def("PauseSimulation", &SimulatorRunner::PauseSimulation)
      .def("UnpauseSimulation", &SimulatorRunner::UnpauseSimulation)
      .def("RequestSimulationStepExecution",
           &SimulatorRunner::RequestSimulationStepExecution)
      .def("get_stats", &SimulatorRunner::get_stats);

  py::class_<RoadBuilder<double>>(m, "RoadBuilder")
      .def(py::init<AutomotiveSimulator<double>*>())
      .def(py::init<AutomotiveSimulator<double>*, double, double>())
      .def("AddDragway", &RoadBuilder<double>::AddDragway)
      .def("AddOnramp", &RoadBuilder<double>::AddOnramp)
      .def("AddMonolaneFromFile", &RoadBuilder<double>::AddMonolaneFromFile)
      .def("AddMultilaneFromFile", &RoadBuilder<double>::AddMultilaneFromFile);

  py::class_<AutomotiveSimulator<double>>(m, "AutomotiveSimulator")
      .def(py::init(
          [](void) { return std::make_unique<AutomotiveSimulator<double>>(); }))
      .def("Start", &AutomotiveSimulator<double>::Start)
      .def("AddAgent", &AutomotiveSimulator<double>::AddAgent)
      //      .def("AddAgent",
      //           py::overload_cast<
      //               std::unique_ptr<delphyne::AgentBase<double>>>(
      //               &AutomotiveSimulator<double>::AddAgent))
      .def("AddLoadableAgent",
           py::overload_cast<
               const std::string&, const std::string&,
               std::unique_ptr<drake::systems::BasicVector<double>>,
               const RoadGeometry*>(
               &AutomotiveSimulator<double>::AddLoadableAgent))
      .def("AddLoadableAgent",
           py::overload_cast<
               const std::string&, const std::string&,
               std::unique_ptr<drake::systems::BasicVector<double>>,
               const RoadGeometry*, std::unique_ptr<AgentPluginParams>>(
               &AutomotiveSimulator<double>::AddLoadableAgent))
      .def("AddLoadableAgent",
           py::overload_cast<
               const std::string&, const std::string&, const std::string&,
               std::unique_ptr<drake::systems::BasicVector<double>>,
               const RoadGeometry*, std::unique_ptr<AgentPluginParams>>(
               &AutomotiveSimulator<double>::AddLoadableAgent));
}

}  // namespace
