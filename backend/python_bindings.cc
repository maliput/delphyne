// Copyright 2017 Toyota Research Institute

#include <memory>

#include "backend/automotive_simulator.h"
#include "backend/linb-any"
#include "backend/road_builder.h"
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
using drake::automotive::SimpleCarState;
using drake::maliput::api::RoadGeometry;

namespace {
PYBIND11_MODULE(python_bindings, m) {
  // TODO(basicNew): Import these from Drake. We are currently defining
  // VectorBase binding so we can use BasicVector (see below), but all these
  // bindings are already defined in Drake, so we should import and use them
  // directly instead of redefining.
  py::class_<VectorBase<double>>(m, "VectorBase")
      .def("CopyToVector", &VectorBase<double>::CopyToVector)
      .def("SetFromVector", &VectorBase<double>::SetFromVector)
      .def("size", &VectorBase<double>::size);

  // TODO(basicNew): Import these from Drake. We are currently defining
  // BasicVector binding so we can use SimpleCarState (see below), but all these
  // bindings are already defined in Drake, so we should import and use them
  // directly instead of redefining.
  py::class_<BasicVector<double>, VectorBase<double>>(m, "BasicVector")
      .def(py::init<int>());

  // TODO(basicNew): Import these from Drake. We are currently defining
  // SimpleCarState so we can use it as a parameter of the  SimulatorRunner,
  // but this class is already defined in Drake, so we should import and use
  // it directly instead of redefining.
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
      .def("AddMonolaneFromFile", &RoadBuilder<double>::AddMonolaneFromFile);

  py::class_<AutomotiveSimulator<double>>(m, "AutomotiveSimulator")
      .def(py::init(
          [](void) { return std::make_unique<AutomotiveSimulator<double>>(); }))
      .def("Start", &AutomotiveSimulator<double>::Start)
      .def("AddLoadableCar", &AutomotiveSimulator<double>::AddLoadableCar)
      .def("AddPriusSimpleCar", &AutomotiveSimulator<double>::AddPriusSimpleCar)
      .def("AddMobilControlledSimpleCar",
           &AutomotiveSimulator<double>::AddMobilControlledSimpleCar);
}

}  // namespace
