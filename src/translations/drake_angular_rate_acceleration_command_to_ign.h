// Copyright 2020 Toyota Research Institute

#pragma once

#include <cstdint>

#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/automotive_angular_rate_acceleration_command.pb.h"
#include "gen/angular_rate_acceleration_command.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

/// @brief A system that translates Drake driving command messages to ignition
/// driving command messages.
class DrakeAngularRateAccelerationCommandToIgn
    : public DrakeToIgn<AngularRateAccelerationCommand<double>,
                        ignition::msgs::AutomotiveAngularRateAccelerationCommand> {
 public:
  DrakeAngularRateAccelerationCommandToIgn();

 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(const AngularRateAccelerationCommand<double>& drake_message,
                               ignition::msgs::AutomotiveAngularRateAccelerationCommand* ign_message,
                               int64_t time_ms) const override;
};

}  // namespace delphyne
