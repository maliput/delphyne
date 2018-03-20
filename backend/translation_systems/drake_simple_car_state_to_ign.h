// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/simple_car.h"

#include "ignition/msgs.hh"

#include "protobuf/simple_car_state.pb.h"

#include "backend/system.h"
#include "backend/translation_systems/drake_to_ign.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

/// @brief A system that translates Drake simple car state messages to ignition
/// simple car state messages.
class DELPHYNE_BACKEND_VISIBLE DrakeSimpleCarStateToIgn
    : public DrakeToIgn<drake::automotive::SimpleCarState<double>,
                        ignition::msgs::SimpleCarState> {
 public:
  DrakeSimpleCarStateToIgn();

 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(
      const drake::automotive::SimpleCarState<double>& drake_message,
      ignition::msgs::SimpleCarState* ign_message,
      int64_t time_ms) const override;
};

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
