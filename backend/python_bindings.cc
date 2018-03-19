// Copyright 2017 Toyota Research Institute

#include <memory>

#include "backend/automotive_simulator.h"
#include "backend/road_builder.h"
#include "backend/simulation_runner.h"

#include <drake/common/find_resource.h>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

using std::unique_ptr;

using delphyne::backend::AutomotiveSimulator;
using delphyne::backend::RoadBuilder;
using delphyne::backend::SimulatorRunner;
using drake::automotive::SimpleCarState;

// Since we are not yet exporting the AutomotiveSimulator class we need to
// provide a ready-to-run SimulatorRunner. To do so we create a parameterless
// constructor that sets up a simulation runner with a prius car in it. As we
// keep adding python bindings to C++ classes this code will be moved to the
// python scripts that launches the simulation.

namespace {
PYBIND11_MODULE(python_bindings, m) {
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
      .def("IsRunning", &SimulatorRunner::IsRunning)
      .def("AddStepCallback", &SimulatorRunner::AddStepCallback)
      .def("RunSimulationStep", &SimulatorRunner::RunSimulationStep)
      .def("IsPaused", &SimulatorRunner::IsPaused)
      .def("Pause", &SimulatorRunner::Pause)
      .def("RequestStep", &SimulatorRunner::RequestStep)
      .def("Unpause", &SimulatorRunner::Unpause);

  py::class_<RoadBuilder<double>>(m, "RoadBuilder")
      .def(py::init<AutomotiveSimulator<double>*>())
      .def(py::init<AutomotiveSimulator<double>*, double, double>())
      .def("AddDragway", &RoadBuilder<double>::AddDragway)
      .def("AddOnramp", &RoadBuilder<double>::AddOnramp)
      .def("LoadMonolane", &RoadBuilder<double>::LoadMonolane);

  py::class_<AutomotiveSimulator<double>>(m, "AutomotiveSimulator")
      .def(py::init(
          [](void) { return std::make_unique<AutomotiveSimulator<double>>(); }))
      .def("Start", &AutomotiveSimulator<double>::Start)
      .def("AddPriusSimpleCar", &AutomotiveSimulator<double>::AddPriusSimpleCar)
      .def("AddMobilControlledSimpleCar",
           &AutomotiveSimulator<double>::AddMobilControlledSimpleCar);
  // TODO(mikaelarguedas) bind more method depending on what we need
  // Needs drake::automotive::Curve2<double>&
  // .def("AddPriusTrajectoryCar",
  // &AutomotiveSimulator<double>::AddPriusTrajectoryCar)
  // Needs:
  //  - drake::automotive::LaneDirection
  //  - drake::automotive::MaliputRailcarParams<T>
  //  - drake::automotive::MaliputRailcarState<T>
  // .def("AddPriusMaliputRailcar",
  // &AutomotiveSimulator<double>::AddPriusMaliputRailcar)
  // Needs:
  //  - drake::automotive::LaneDirection
  //  - drake::automotive::MaliputRailcarParams<T>
  //  - drake::automotive::MaliputRailcarState<T>
  // .def("AddIdmControlledPriusMaliputRailcar",
  // &AutomotiveSimulator<double>::AddIdmControlledPriusMaliputRailcar)
  // .def("SetMaliputRailcarAccelerationCommand",
  // &AutomotiveSimulator<double>::SetMaliputRailcarAccelerationCommand)
  // Needs drake::maliput::api::RoadGeometry binding
  // .def("SetRoadGeometry", &AutomotiveSimulator<double>::SetRoadGeometry)
  // Needs drake::maliput::api::Lane binding
  // .def("FindLane", &AutomotiveSimulator<double>::FindLane)
  // Needs drake::systems::System<T> binding
  // .def("GetDiagramSystemByName",
  // &AutomotiveSimulator<double>::GetDiagramSystemByName)
  // .def("Build", &AutomotiveSimulator<double>::Build)
  // .def("GetDiagram", &AutomotiveSimulator<double>::GetDiagram)
  // .def("StepBy", &AutomotiveSimulator<double>::StepBy)
  // Needs drake::systems::rendering::PoseBundle<T> binding
  // .def("GetCurrentPoses", &AutomotiveSimulator<double>::GetCurrentPoses)
  // TODO(mikaelarguedas) Submit this to upstream Drake
  py::class_<SimpleCarState<double>>(m, "SimpleCarState")
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
}

}  // namespace
