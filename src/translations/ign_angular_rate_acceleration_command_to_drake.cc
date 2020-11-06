// Copyright 2020 Toyota Research Institute

#include "translations/ign_angular_rate_acceleration_command_to_drake.h"

namespace delphyne {

void IgnAngularRateAccelerationCommandToDrake::DoIgnToDrakeTranslation(
    const ignition::msgs::AutomotiveAngularRateAccelerationCommand& ign_message,
    AngularRateAccelerationCommand<double>* drake_message) const {
  DELPHYNE_VALIDATE(drake_message != nullptr, std::invalid_argument, "Drake message pointer must not be null");

  drake_message->set_angular_rate(ign_message.omega());
  drake_message->set_acceleration(ign_message.acceleration());
}

}  // namespace delphyne
