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

#include <boost/python.hpp>
#include <drake/common/find_resource.h>

#include "backend/simulation_runner.h"

using delphyne::backend::SimulatorRunner;

// Since we are not yet exporting the AutomotiveSimulator class we need to
// provide a ready-to-run SimulatorRunner. To do so we create a parameterless
// constructor that sets up a simulation runner with a prius car in it. As we
// keep adding python bindings to C++ classes this code will be moved to the
// python scripts that launches the simulation.
static std::shared_ptr<SimulatorRunner> simulatorRunnerFactory() {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");

  auto simulator =
      std::make_unique<drake::automotive::AutomotiveSimulator<double>>();

  // Add a Prius car.
  drake::automotive::SimpleCarState<double> state;
  state.set_y(0.0);
  simulator->AddPriusSimpleCar("0", "DRIVING_COMMAND_0", state);

  // Instantiate the simulator runner and pass the simulator.
  auto timeStep = 0.001;
  return std::make_shared<SimulatorRunner>(std::move(simulator), timeStep);
}

BOOST_PYTHON_MODULE(simulation_runner_py) {
  boost::python::class_<SimulatorRunner, boost::noncopyable>(
      "SimulatorRunner", boost::python::no_init)
      .def("__init__", boost::python::make_constructor(simulatorRunnerFactory))
      .def("start", &SimulatorRunner::Start);
}
