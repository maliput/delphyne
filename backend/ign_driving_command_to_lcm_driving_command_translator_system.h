// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/gen/driving_command.h"

#include "ignition/msgs.hh"

#include "protobuf/automotive_driving_command.pb.h"

#include "backend/ign_to_lcm_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates ignition driving command messages to LCM
/// driving command messages.
class DELPHYNE_BACKEND_VISIBLE
    IgnDrivingCommandToLcmDrivingCommandTranslatorSystem
    : public IgnToLcmTranslatorSystem<
          ignition::msgs::AutomotiveDrivingCommand,
          drake::automotive::DrivingCommand<double>> {
 public:
  /// @brief Default constructor. @see IgnToLcmTranslatorSystem::InitPorts.
  IgnDrivingCommandToLcmDrivingCommandTranslatorSystem();

 protected:
  // @brief @see IgnToLcmTranslatorSystem::DoIgnToLcmTranslation.
  void DoIgnToLcmTranslation(
      const ignition::msgs::AutomotiveDrivingCommand& ign_message,
      drake::automotive::DrivingCommand<double>* lcm_message) const override;
};

}  // namespace backend
}  // namespace delphyne
