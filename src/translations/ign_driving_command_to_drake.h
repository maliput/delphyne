// Copyright 2018 Toyota Research Institute

#pragma once

#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/automotive_driving_command.pb.h"
#include "gen/driving_command.h"
#include "translations/ign_to_drake.h"

namespace delphyne {

/// @brief A system that translates ignition driving command messages to Drake
/// driving command messages.
class IgnDrivingCommandToDrake
    : public IgnToDrake<ignition::msgs::AutomotiveDrivingCommand,
                        DrivingCommand<double>> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::AutomotiveDrivingCommand& ign_message,
      DrivingCommand<double>* drake_message) const override;
};

}  // namespace delphyne
