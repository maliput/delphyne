#ifndef DELPHYNE_BRIDGE_SERVICETOCHANNELTRANSLATION_HH_
#define DELPHYNE_BRIDGE_SERVICETOCHANNELTRANSLATION_HH_

#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_viewer_command.hpp"

namespace delphyne {
namespace bridge {

/// \brief Generates an LCM message aimed to
/// tell drake that the visualizer is ready
drake::lcmt_viewer_command convertServiceToMsg(ignition::msgs::Empty request);

}  // namespace bridge
}  // namespace delphyne

#endif
