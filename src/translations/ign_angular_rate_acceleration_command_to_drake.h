// Copyright 2020 Toyota Research Institute

#pragma once

#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/automotive_angular_rate_acceleration_command.pb.h"
#include "gen/angular_rate_acceleration_command.h"
#include "translations/ign_to_drake.h"

namespace delphyne {

/// @brief A system that translates ignition angular rate / acceleration command messages to Drake
/// angular rate / acceleration command messages.
class IgnAngularRateAccelerationCommandToDrake
    : public IgnToDrake<ignition::msgs::AutomotiveAngularRateAccelerationCommand,
                        AngularRateAccelerationCommand<double>> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(const ignition::msgs::AutomotiveAngularRateAccelerationCommand& ign_message,
                               AngularRateAccelerationCommand<double>* drake_message) const override;
};

}  // namespace delphyne
