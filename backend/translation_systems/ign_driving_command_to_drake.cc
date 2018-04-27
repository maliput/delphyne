// Copyright 2018 Toyota Research Institute

#include "backend/translation_systems/ign_driving_command_to_drake.h"

namespace delphyne {
namespace translation_systems {

void IgnDrivingCommandToDrake::DoIgnToDrakeTranslation(
    const ignition::msgs::AutomotiveDrivingCommand& ign_message,
    drake::automotive::DrivingCommand<double>* drake_message) const {
  DELPHYNE_DEMAND(drake_message != nullptr);

  drake_message->set_steering_angle(ign_message.theta());
  drake_message->set_acceleration(ign_message.acceleration());
}

}  // namespace translation_systems
}  // namespace delphyne
