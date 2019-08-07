// Copyright 2018 Toyota Research Institute

#include "translations/drake_driving_command_to_ign.h"

#include <map>
#include <string>

#include "translations/time_conversion.h"

namespace delphyne {

DrakeDrivingCommandToIgn::DrakeDrivingCommandToIgn() : DrakeToIgn(DrivingCommandIndices::kNumCoordinates) {}

void DrakeDrivingCommandToIgn::DoDrakeToIgnTranslation(const DrivingCommand<double>& drake_message,
                                                       ignition::msgs::AutomotiveDrivingCommand* ign_message,
                                                       int64_t time_ms) const {
  DELPHYNE_VALIDATE(ign_message != nullptr, std::invalid_argument, "Ignition message pointer must not be null");

  ign_message->mutable_time()->CopyFrom(MillisToIgnitionTime(time_ms));
  ign_message->set_theta(drake_message.steering_angle());
  ign_message->set_acceleration(drake_message.acceleration());
}

}  // namespace delphyne
