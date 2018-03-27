// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/gen/driving_command.h"

#include "ignition/msgs.hh"

#include "protobuf/automotive_driving_command.pb.h"

#include "backend/system.h"
#include "backend/translation_systems/ign_to_drake.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

/// @brief A system that translates ignition driving command messages to Drake
/// driving command messages.
class DELPHYNE_BACKEND_VISIBLE IgnDrivingCommandToDrake
    : public IgnToDrake<ignition::msgs::AutomotiveDrivingCommand,
                        drake::automotive::DrivingCommand<double>> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::AutomotiveDrivingCommand& ign_message,
      drake::automotive::DrivingCommand<double>* drake_message) const override;
};

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
