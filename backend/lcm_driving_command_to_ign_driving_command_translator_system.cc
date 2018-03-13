// Copyright 2018 Toyota Research Institute

#include <map>
#include <string>

#include "backend/lcm_driving_command_to_ign_driving_command_translator_system.h"

namespace delphyne {
namespace backend {

LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::
    LcmDrivingCommandToIgnDrivingCommandTranslatorSystem() {
  InitPorts();
}

void LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::
    DoLcmToIgnTranslation(
        const drake::automotive::DrivingCommand<double>& lcm_message,
        ignition::msgs::AutomotiveDrivingCommand* ign_message) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  auto time_ms = static_cast<int64_t>(translation_context->get_time()) * 1000;

  LcmTimestampToIgnition(time_ms, ign_message->mutable_time());
  ign_message->set_theta(lcm_message.steering_angle());
  ign_message->set_acceleration(lcm_message.acceleration());
}

int LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::GetVectorSize()
    const {
  return drake::automotive::DrivingCommandIndices::kNumCoordinates;
}

}  // namespace backend
}  // namespace delphyne
