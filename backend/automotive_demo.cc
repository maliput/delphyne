// Copyright 2017 Toyota Research Institute

#include <cstdlib>
#include <memory>
#include <string>

#include "backend/simulation_runner.h"

#include <drake/automotive/automotive_simulator.h>
#include <drake/common/find_resource.h>

namespace delphyne {
namespace backend {
namespace {

// Generates a channel name based on a given string.
std::string MakeChannelName(const std::string& name) {
  const std::string default_prefix{"DRIVING_COMMAND"};
  if (name.empty()) {
    return default_prefix;
  }
  return default_prefix + "_" + name;
}

int main(int argc, char* argv[]) {
  // Instantiates a simulator.
  auto simulator =
      std::make_unique<delphyne::backend::AutomotiveSimulator<double>>();

  // Adds a Prius car.
  drake::automotive::SimpleCarState<double> state;
  state.set_y(0.0);
  simulator->AddPriusSimpleCar("0", MakeChannelName("0"), state);

  // Adds a Loadable Prius Simple car.
  drake::automotive::SimpleCarState<double> state2;
  state2.set_y(4.0);
  std::map<std::string, linb::any> simple_params;
  if (simulator->AddLoadableCar("LoadablePriusSimpleCar", simple_params, "1",
                                &state2) < 0) {
    return 1;
  }

  // Instantiates the simulator runner and starts it.
  const double kTimeStep = 0.001;
  delphyne::backend::SimulatorRunner prius_sim_runner(std::move(simulator),
                                                      kTimeStep);
  prius_sim_runner.Start();

  // Zzzzzz.
  delphyne::backend::WaitForShutdown();
  return 0;
}

}  // namespace
}  // namespace backend
}  // namespace delphyne

int main(int argc, char* argv[]) { return delphyne::backend::main(argc, argv); }
