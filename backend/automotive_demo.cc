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

#include <cstdlib>
#include <memory>
#include <string>
#include <drake/common/find_resource.h>

#include "backend/simulation_runner.h"

//////////////////////////////////////////////////
std::string MakeChannelName(const std::string& _name) {
  const std::string defaultPrefix{"DRIVING_COMMAND"};
  if (_name.empty()) {
    return defaultPrefix;
  }
  return defaultPrefix + "_" + _name;
}

//////////////////////////////////////////////////
int main(int argc, char* argv[]) {
  // Enable to resolve relative path to resources on AddPriusSimpleCar
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");

  // Instantiate a simulator.
  auto simulator =
      std::make_unique<delphyne::backend::AutomotiveSimulator<double>>();

  // Add a Prius car.
  drake::automotive::SimpleCarState<double> state;
  state.set_y(0.0);
  simulator->AddPriusSimpleCar("0", MakeChannelName("0"), state);

  // Instantiate the simulator runner and pass the simulator.
  const double kTimeStep = 0.001;
  delphyne::backend::SimulatorRunner priusSimRunner(std::move(simulator),
                                                    kTimeStep);
  priusSimRunner.Start();

  // Zzzzzz.
  delphyne::backend::WaitForShutdown();
}
