// Copyright 2017 Toyota Research Institute

#include <memory>

#include "backend/automotive_simulator.h"
#include "backend/interactive_simulation_stats.h"
#include "backend/linb-any"
#include "backend/road_builder.h"
#include "backend/simulation_run_stats.h"
#include "backend/simulation_runner.h"

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
using drake::maliput::api::RoadGeometry;
using drake::systems::BasicVector;
using drake::systems::VectorBase;

namespace {
PYBIND11_MODULE(python_bindings, m) {
  // TODO(apojomovsky): Import this from Drake. Tracked in delphyne's #339.
  // Depends on drake's #8096 to be solved before we can actually replace
  // this binding with it, since it currently lacks of constructors/methods.
  // We are currently defining SimpleCarState so we can use it as a parameter
  // of the SimulatorRunner.
  py::class_<SimpleCarState<double>, BasicVector<double>>(m, "SimpleCarState")
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

  // TODO(basicNew): Properly fill this binding and submit to Drake. We are
  // currently defining just the class name so we can use it to pass road
  // geometry pointers around.
  py::class_<RoadGeometry>(m, "RoadGeometry");

  // TODO(basicNew): Properly fill this binding with the remaining methods and
  // overloaded constructors.
  py::class_<linb::any>(m, "Any")
      .def(py::init<bool&&>())
      .def(py::init<const RoadGeometry*&&>());

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

  py::class_<AutomotiveSimulator<double>>(m, "AutomotiveSimulator")
      .def(py::init(
          [](void) { return std::make_unique<AutomotiveSimulator<double>>(); }))
      .def("Start", &AutomotiveSimulator<double>::Start)
      .def("AddLoadableCar", &AutomotiveSimulator<double>::AddLoadableCar)
      .def("AddPriusSimpleCar",
           &AutomotiveSimulator<double>::AddPriusSimpleCar);
}

}  // namespace
