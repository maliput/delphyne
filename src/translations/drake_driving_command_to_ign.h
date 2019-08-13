// Copyright 2018 Toyota Research Institute

#pragma once

#include <cstdint>

#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/automotive_driving_command.pb.h"
#include "gen/driving_command.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

/// @brief A system that translates Drake driving command messages to ignition
/// driving command messages.
class DrakeDrivingCommandToIgn : public DrakeToIgn<DrivingCommand<double>, ignition::msgs::AutomotiveDrivingCommand> {
 public:
  DrakeDrivingCommandToIgn();

 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(const DrivingCommand<double>& drake_message,
                               ignition::msgs::AutomotiveDrivingCommand* ign_message, int64_t time_ms) const override;
};

}  // namespace delphyne
