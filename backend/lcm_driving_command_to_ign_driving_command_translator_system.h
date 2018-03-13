// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/gen/driving_command.h"

#include "ignition/msgs.hh"

#include "protobuf/automotive_driving_command.pb.h"

#include "backend/lcm_to_ign_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates LCM driving command messages to ignition
/// driving command messages.
class DELPHYNE_BACKEND_VISIBLE
    LcmDrivingCommandToIgnDrivingCommandTranslatorSystem
    : public LcmToIgnTranslatorSystem<
          drake::automotive::DrivingCommand<double>,
          ignition::msgs::AutomotiveDrivingCommand> {
 public:
  /// @brief Default constructor. @see LcmToIgnTranslatorSystem::InitPorts.
  LcmDrivingCommandToIgnDrivingCommandTranslatorSystem();

 protected:
  // @brief @see LcmToIgnTranslatorSystem::DoLcmToIgnTranslation.
  void DoLcmToIgnTranslation(
      const drake::automotive::DrivingCommand<double>& lcm_message,
      ignition::msgs::AutomotiveDrivingCommand* ign_message) const override;

  int GetVectorSize() const override;
};

}  // namespace backend
}  // namespace delphyne
