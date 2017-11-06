#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/monolane_onramp_merge.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"

using namespace drake;
using namespace automotive;

namespace delphyne {
namespace backend {
namespace {

std::string MakeChannelName(const std::string& name) {
  const std::string default_prefix{"DRIVING_COMMAND"};
  if (name.empty()) {
    return default_prefix;
  }
  return default_prefix + "_" + name;
}

int main(int argc, char* argv[]) {
  // Enable to resolve relative path to resources on AddPriusSimpleCar
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                        "/share/drake");

  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  SimpleCarState<double> state;
  state.set_y(0.0);
  simulator.get()->AddPriusSimpleCar("0", MakeChannelName("0"), state);

  simulator->Start(1.0);
  simulator->StepBy(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace
}  // namespace backend
}  // namespace delphyne

int main(int argc, char* argv[]) { return delphyne::backend::main(argc, argv); }
