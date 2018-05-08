// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/gen/driving_command.h"

#include <cstdint>

#include "ignition/msgs.hh"

#include "delphyne/protobuf/automotive_driving_command.pb.h"

#include "backend/system.h"
#include "backend/translation_systems/drake_to_ign.h"

namespace delphyne {

/// @brief A system that translates Drake driving command messages to ignition
/// driving command messages.
class DrakeDrivingCommandToIgn
    : public DrakeToIgn<drake::automotive::DrivingCommand<double>,
                        ignition::msgs::AutomotiveDrivingCommand> {
 public:
  DrakeDrivingCommandToIgn();

 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(
      const drake::automotive::DrivingCommand<double>& drake_message,
      ignition::msgs::AutomotiveDrivingCommand* ign_message,
      int64_t time_ms) const override;
};

}  // namespace delphyne
