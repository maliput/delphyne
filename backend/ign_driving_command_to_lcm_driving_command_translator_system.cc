// Copyright 2018 Toyota Research Institute

#include "backend/ign_driving_command_to_lcm_driving_command_translator_system.h"

namespace delphyne {
namespace backend {

IgnDrivingCommandToLcmDrivingCommandTranslatorSystem::
    IgnDrivingCommandToLcmDrivingCommandTranslatorSystem() {
  InitPorts();
}

void IgnDrivingCommandToLcmDrivingCommandTranslatorSystem::
    DoIgnToLcmTranslation(
        const ignition::msgs::AutomotiveDrivingCommand& ign_message,
        drake::automotive::DrivingCommand<double>* lcm_message) const {
  DELPHYNE_DEMAND(lcm_message != nullptr);

  lcm_message->set_steering_angle(ign_message.theta());
  lcm_message->set_acceleration(ign_message.acceleration());
}

}  // namespace backend
}  // namespace delphyne
