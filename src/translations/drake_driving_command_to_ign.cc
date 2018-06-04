// Copyright 2018 Toyota Research Institute

#include "translations/drake_driving_command_to_ign.h"

#include <map>
#include <string>

#include "backend/time_conversion.h"

namespace delphyne {

DrakeDrivingCommandToIgn::DrakeDrivingCommandToIgn()
    : DrakeToIgn(drake::automotive::DrivingCommandIndices::kNumCoordinates) {}

void DrakeDrivingCommandToIgn::DoDrakeToIgnTranslation(
    const drake::automotive::DrivingCommand<double>& drake_message,
    ignition::msgs::AutomotiveDrivingCommand* ign_message,
    int64_t time_ms) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  ign_message->mutable_time()->CopyFrom(MillisToIgnitionTime(time_ms));
  ign_message->set_theta(drake_message.steering_angle());
  ign_message->set_acceleration(drake_message.acceleration());
}

}  // namespace delphyne
