// Copyright 2018 Toyota Research Institute

#include <map>
#include <string>

#include "backend/lcm_driving_command_to_ign_driving_command_translator_system.h"

namespace delphyne {
namespace backend {

LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::
    LcmDrivingCommandToIgnDrivingCommandTranslatorSystem() {
  InitPorts(drake::automotive::DrivingCommandIndices::kNumCoordinates);
}

void LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::
    DoLcmToIgnTranslation(
        const drake::automotive::DrivingCommand<double>& lcm_message,
        ignition::msgs::AutomotiveDrivingCommand* ign_message,
        int64_t time_ms) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  LcmTimestampToIgnition(time_ms, ign_message->mutable_time());
  ign_message->set_theta(lcm_message.steering_angle());
  ign_message->set_acceleration(lcm_message.acceleration());
}

}  // namespace backend
}  // namespace delphyne
