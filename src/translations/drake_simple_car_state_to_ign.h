// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/simple_car.h"

#include <cstdint>

#include "ignition/msgs.hh"

#include "backend/system.h"
#include "translations/drake_to_ign.h"

#include "delphyne/protobuf/simple_car_state.pb.h"

namespace delphyne {

/// @brief A system that translates Drake simple car state messages to ignition
/// simple car state messages.
class DrakeSimpleCarStateToIgn
    : public DrakeToIgn<drake::automotive::SimpleCarState<double>,
                        ignition::msgs::SimpleCarState> {
 public:
  DrakeSimpleCarStateToIgn();
  virtual ~DrakeSimpleCarStateToIgn() = default;

 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(
      const drake::automotive::SimpleCarState<double>& drake_message,
      ignition::msgs::SimpleCarState* ign_message,
      int64_t time_ms) const override;
};

}  // namespace delphyne