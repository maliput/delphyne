// Copyright 2018 Toyota Research Institute

#include "backend/ign_driving_command_to_drake_driving_command_translator_system.h"

namespace delphyne {
namespace backend {

void IgnDrivingCommandToDrakeDrivingCommandTranslatorSystem::
    DoIgnToDrakeTranslation(
        const ignition::msgs::AutomotiveDrivingCommand& ign_message,
        drake::automotive::DrivingCommand<double>* drake_message) const {
  DELPHYNE_DEMAND(drake_message != nullptr);

  drake_message->set_steering_angle(ign_message.theta());
  drake_message->set_acceleration(ign_message.acceleration());
}

}  // namespace backend
}  // namespace delphyne
