// Copyright 2017 Toyota Research Institute

#include <memory>

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

#include "../include/delphyne/linb-any"

namespace py = pybind11;

using std::unique_ptr;

using delphyne::AutomotiveSimulator;
using delphyne::RoadBuilder;
using delphyne::SimulatorRunner;
using delphyne::InteractiveSimulationStats;
using drake::automotive::LaneDirection;
using drake::automotive::MaliputRailcarParams;
using drake::automotive::MaliputRailcarState;
using drake::maliput::api::RoadGeometry;
using drake::systems::BasicVector;

namespace {
PYBIND11_MODULE(python_bindings, m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.maliput.api");

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

  // TODO(basicNew): Properly fill this binding with the remaining methods and
  // overloaded constructors.
  // Note: Since we are using linb::any in combination with bare pointers to
  // pass generic parameters to the loadable agents module, we have to make
  // sure python doesn't garbage collect temp objects while they are being
  // used on the C++ side, hence the `py::keep_alive`. See
  // http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive
  py::class_<linb::any>(m, "Any")
      .def(py::init<bool&&>())
      // Keep alive, ownership: `self` keeps `RoadGeometry` alive.
      .def(py::init<const RoadGeometry*&&>(), py::keep_alive<1, 2>())
      // Keep alive, ownership: `self` keeps `LaneDirection` alive.
      .def(py::init<LaneDirection*&&>(), py::keep_alive<1, 2>())
      // Keep alive, ownership: `self` keeps `MaliputRailcarParams` alive.
      .def(py::init<MaliputRailcarParams<double>*&&>(), py::keep_alive<1, 2>());

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

  // Note: Since AddLoadableAgent uses a map of (string, linb::any) in
  // combination with bare pointers to pass generic parameters, we have to make
  // sure python doesn't garbage collect temp objects while they are being
  // used on the C++ side, hence the `py::keep_alive`. See
  // http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive
  py::class_<AutomotiveSimulator<double>>(m, "AutomotiveSimulator")
      .def(py::init(
          [](void) { return std::make_unique<AutomotiveSimulator<double>>(); }))
      .def("Start", &AutomotiveSimulator<double>::Start)
      .def("AddLoadableAgent",
           py::overload_cast<
               const std::string&, const std::map<std::string, linb::any>&,
               const std::string&,
               std::unique_ptr<drake::systems::BasicVector<double>>>(
               &AutomotiveSimulator<double>::AddLoadableAgent),
           py::keep_alive<1, 3>())  // Keep alive, ownership: `self` keeps
                                    // `parameters` alive.
      .def("AddLoadableAgent",
           py::overload_cast<
               const std::string&, const std::string&,
               const std::map<std::string, linb::any>&, const std::string&,
               std::unique_ptr<drake::systems::BasicVector<double>>>(
               &AutomotiveSimulator<double>::AddLoadableAgent),
           py::keep_alive<1, 3>());  // Keep alive, ownership: `self` keeps
                                     // `parameters` alive.
}

}  // namespace
