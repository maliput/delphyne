// Copyright 2018 Toyota Research Institute

#include "backend/translation_systems/drake_driving_command_to_ign_driving_command.h"

#include <map>
#include <string>

#include "backend/time_conversion.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

DrakeDrivingCommandToIgnDrivingCommand::DrakeDrivingCommandToIgnDrivingCommand()
    : DrakeToIgn(drake::automotive::DrivingCommandIndices::kNumCoordinates) {}

void DrakeDrivingCommandToIgnDrivingCommand::DoDrakeToIgnTranslation(
    const drake::automotive::DrivingCommand<double>& drake_message,
    ignition::msgs::AutomotiveDrivingCommand* ign_message,
    int64_t time_ms) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  ign_message->mutable_time()->CopyFrom(MillisToIgnitionTime(time_ms));
  ign_message->set_theta(drake_message.steering_angle());
  ign_message->set_acceleration(drake_message.acceleration());
}

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
