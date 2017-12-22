// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>

#include <drake/common/find_resource.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include "backend/automotive_simulator.h"
#include "backend/simulation_runner.h"

namespace py = pybind11;

using delphyne::backend::AutomotiveSimulator;
using delphyne::backend::SimulatorRunner;
using drake::automotive::SimpleCarState;

// Since we are not yet exporting the AutomotiveSimulator class we need to
// provide a ready-to-run SimulatorRunner. To do so we create a parameterless
// constructor that sets up a simulation runner with a prius car in it. As we
// keep adding python bindings to C++ classes this code will be moved to the
// python scripts that launches the simulation.

namespace {
PYBIND11_MODULE(simulation_runner_py, m) {
  py::class_<SimulatorRunner>(m, "SimulatorRunner")
    .def(py::init([](std::unique_ptr<AutomotiveSimulator<double>> simulator, double time_step) {
      return std::make_unique<SimulatorRunner>(std::move(simulator), time_step);
    }))
    .def("Start", &SimulatorRunner::Start)
    .def("Stop", &SimulatorRunner::Stop)
    .def("AddStepCallback", &SimulatorRunner::AddStepCallback);
  ;
  py::class_<AutomotiveSimulator<double>, std::unique_ptr<AutomotiveSimulator<double>>>(m, "AutomotiveSimulator")
    .def(py::init([](void) {
        return std::make_unique<AutomotiveSimulator<double>>();
    }))
    .def("Start", &AutomotiveSimulator<double>::Start)
    .def("AddPriusSimpleCar", &AutomotiveSimulator<double>::AddPriusSimpleCar)
    .def("AddMobilControlledSimpleCar", &AutomotiveSimulator<double>::AddMobilControlledSimpleCar)
    // TODO(mikaelarguedas) bind more method depending on what we need
    // .def("AddPriusTrajectoryCar", &AutomotiveSimulator<double>::AddPriusTrajectoryCar)
    // .def("AddPriusMaliputRailcar", &AutomotiveSimulator<double>::AddPriusMaliputRailcar)
    // .def("AddIdmControlledPriusMaliputRailcar", &AutomotiveSimulator<double>::AddIdmControlledPriusMaliputRailcar)
    // .def("SetMaliputRailcarAccelerationCommand", &AutomotiveSimulator<double>::SetMaliputRailcarAccelerationCommand)
  ;
  py::class_<SimpleCarState<double>>(m, "SimpleCarState")
    .def(py::init<>())
    .def_property("x", &SimpleCarState<double>::x, &SimpleCarState<double>::set_x)
    .def_property("y", &SimpleCarState<double>::y, &SimpleCarState<double>::set_y)
    .def_property("heading", &SimpleCarState<double>::heading, &SimpleCarState<double>::set_heading)
    .def_property("velocity", &SimpleCarState<double>::velocity, &SimpleCarState<double>::set_velocity)
    .def("get_coordinates_names", &SimpleCarState<double>::GetCoordinateNames)
  ;
}

}  // namespace
