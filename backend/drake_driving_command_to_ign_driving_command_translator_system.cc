// Copyright 2018 Toyota Research Institute

#include <map>
#include <string>

#include "backend/drake_driving_command_to_ign_driving_command_translator_system.h"

namespace delphyne {
namespace backend {

DrakeDrivingCommandToIgnDrivingCommandTranslatorSystem::
    DrakeDrivingCommandToIgnDrivingCommandTranslatorSystem()
    : DrakeToIgnTranslatorSystem(
          drake::automotive::DrivingCommandIndices::kNumCoordinates) {}

void DrakeDrivingCommandToIgnDrivingCommandTranslatorSystem::
    DoDrakeToIgnTranslation(
        const drake::automotive::DrivingCommand<double>& drake_message,
        ignition::msgs::AutomotiveDrivingCommand* ign_message,
        int64_t time_ms) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  LcmTimestampToIgnition(time_ms, ign_message->mutable_time());
  ign_message->set_theta(drake_message.steering_angle());
  ign_message->set_acceleration(drake_message.acceleration());
}

}  // namespace backend
}  // namespace delphyne
