// Copyright 2017 Toyota Research Institute

#include <memory>

#include "backend/automotive_simulator.h"
#include "backend/interactive_simulation_stats.h"
#include "backend/linb-any"
#include "backend/road_builder.h"
#include "backend/simulation_run_stats.h"
#include "backend/simulation_runner.h"

#include <drake/automotive/gen/maliput_railcar_params.h>
#include <drake/common/find_resource.h>
#include <drake/systems/framework/basic_vector.h>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using std::unique_ptr;

using delphyne::backend::AutomotiveSimulator;
using delphyne::backend::RoadBuilder;
using delphyne::backend::SimulatorRunner;
using delphyne::backend::InteractiveSimulationStats;
using delphyne::backend::SimulationRunStats;
using drake::automotive::SimpleCarState;
using drake::automotive::MaliputRailcarState;
using drake::automotive::MaliputRailcarParams;
using drake::automotive::LaneDirection;
using drake::maliput::api::RoadGeometry;
using drake::systems::BasicVector;
using drake::systems::VectorBase;

namespace {
PYBIND11_MODULE(python_bindings, m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.maliput.api");

  // TODO(apojomovsky): Import this from Drake. Tracked in delphyne's #339.
  // Depends on drake's #8096 to be solved before we can actually replace
  // this binding with it, since it currently lacks of constructors/methods.
  // We are currently defining SimpleCarState so we can use it as a parameter
  // of the SimulatorRunner.
  py::class_<SimpleCarState<double>, BasicVector<double>>(m, "SimpleCarState",
                                                          py::module_local())
      .def(py::init<>())
      .def_property("x", &SimpleCarState<double>::x,
                    &SimpleCarState<double>::set_x)
      .def_property("y", &SimpleCarState<double>::y,
                    &SimpleCarState<double>::set_y)
      .def_property("heading", &SimpleCarState<double>::heading,
                    &SimpleCarState<double>::set_heading)
      .def_property("velocity", &SimpleCarState<double>::velocity,
                    &SimpleCarState<double>::set_velocity)
      .def("get_coordinates_names",
           &SimpleCarState<double>::GetCoordinateNames);

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
      .def(py::init<const RoadGeometry*&&>(), py::keep_alive<1, 2>())
      .def(py::init<LaneDirection*&&>(), py::keep_alive<1, 2>())
      .def(py::init<MaliputRailcarParams<double>*&&>(), py::keep_alive<1, 2>());

  py::class_<InteractiveSimulationStats>(m, "InteractiveSimulationStats")
      .def(py::init<>())
      .def("TotalElapsedSimtime",
           &InteractiveSimulationStats::TotalElapsedSimtime)
      .def("TotalElapsedRealtime",
           &InteractiveSimulationStats::TotalElapsedRealtime)
      .def("TotalExecutedSteps",
           &InteractiveSimulationStats::TotalExecutedSteps)
      .def("TotalRuns", &InteractiveSimulationStats::TotalRuns);

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

  // Note: Since AddLoadableCar uses a map of (string, linb::any) in
  // combination with bare pointers to pass generic parameters, we have to make
  // sure python doesn't garbage collect temp objects while they are being
  // used on the C++ side, hence the `py::keep_alive`. See
  // http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive
  py::class_<AutomotiveSimulator<double>>(m, "AutomotiveSimulator")
      .def(py::init(
          [](void) { return std::make_unique<AutomotiveSimulator<double>>(); }))
      .def("Start", &AutomotiveSimulator<double>::Start)
      .def("AddLoadableAgent", &AutomotiveSimulator<double>::AddLoadableAgent,
           py::keep_alive<1, 3>())
      .def("AddPriusSimpleCar",
           &AutomotiveSimulator<double>::AddPriusSimpleCar);
}

}  // namespace
