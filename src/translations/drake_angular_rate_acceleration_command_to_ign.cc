// Copyright 2020 Toyota Research Institute

#include "translations/drake_angular_rate_acceleration_command_to_ign.h"

#include <map>
#include <string>

#include "translations/time_conversion.h"

namespace delphyne {

DrakeAngularRateAccelerationCommandToIgn::DrakeAngularRateAccelerationCommandToIgn()
    : DrakeToIgn(AngularRateAccelerationCommandIndices::kNumCoordinates) {}

void DrakeAngularRateAccelerationCommandToIgn::DoDrakeToIgnTranslation(
    const AngularRateAccelerationCommand<double>& drake_message,
    ignition::msgs::AutomotiveAngularRateAccelerationCommand* ign_message, int64_t time_ms) const {
  DELPHYNE_VALIDATE(ign_message != nullptr, std::invalid_argument, "Ignition message pointer must not be null");

  ign_message->mutable_time()->CopyFrom(MillisToIgnitionTime(time_ms));
  ign_message->set_omega(drake_message.angular_rate());
  ign_message->set_acceleration(drake_message.acceleration());
}

}  // namespace delphyne
