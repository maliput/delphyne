// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/gen/driving_command.h"

#include "ignition/msgs.hh"

#include "protobuf/automotive_driving_command.pb.h"

#include "backend/ign_to_drake_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates ignition driving command messages to Drake
///        driving command messages.
class DELPHYNE_BACKEND_VISIBLE
    IgnDrivingCommandToDrakeDrivingCommandTranslatorSystem
    : public IgnToDrakeTranslatorSystem<
          ignition::msgs::AutomotiveDrivingCommand,
          drake::automotive::DrivingCommand<double>> {
 protected:
  // @brief @see IgnToDrakeTranslatorSystem::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::AutomotiveDrivingCommand& ign_message,
      drake::automotive::DrivingCommand<double>* drake_message) const override;
};

}  // namespace backend
}  // namespace delphyne
