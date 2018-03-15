// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/gen/driving_command.h"

#include "ignition/msgs.hh"

#include "protobuf/automotive_driving_command.pb.h"

#include "backend/drake_to_ign_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

/// @brief A system that translates Drake driving command messages to ignition
///        driving command messages.
class DELPHYNE_BACKEND_VISIBLE DrakeDrivingCommandToIgnDrivingCommand
    : public DrakeToIgn<drake::automotive::DrivingCommand<double>,
                        ignition::msgs::AutomotiveDrivingCommand> {
 public:
  DrakeDrivingCommandToIgnDrivingCommand();

 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(
      const drake::automotive::DrivingCommand<double>& drake_message,
      ignition::msgs::AutomotiveDrivingCommand* ign_message,
      int64_t time_ms) const override;
};

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
