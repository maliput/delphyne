// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/automotive/gen/driving_command.h>

#include <ignition/msgs.hh>

#include "backend/assert.h"
#include "delphyne/protobuf/automotive_driving_command.pb.h"
#include "translations/ign_to_drake.h"

namespace delphyne {

/// @brief A system that translates ignition driving command messages to Drake
/// driving command messages.
class IgnDrivingCommandToDrake
    : public IgnToDrake<ignition::msgs::AutomotiveDrivingCommand,
                        drake::automotive::DrivingCommand<double>> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::AutomotiveDrivingCommand& ign_message,
      drake::automotive::DrivingCommand<double>* drake_message) const override;
};

}  // namespace delphyne
