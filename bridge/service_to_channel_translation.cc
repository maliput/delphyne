#include "bridge/service_to_channel_translation.h"

namespace delphyne {
namespace bridge {

/// \brief Generates an LCM message aimed to
/// tell drake that the visualizer is ready
drake::lcmt_viewer_command convertServiceToMsg(ignition::msgs::Empty request) {
  // Create an empty lcm viewer command message
  drake::lcmt_viewer_command viewerCommand;
  // Fill the message with the content expected by drake
  viewerCommand.command_type = drake::lcmt_viewer_command::STATUS;
  viewerCommand.command_data = "successfully loaded robot";
  return viewerCommand;
}

}  // namespace bridge
}  // namespace delphyne
