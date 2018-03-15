// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/simple_car.h"

#include "ignition/msgs.hh"

#include "protobuf/simple_car_state.pb.h"

#include "backend/drake_to_ign_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates Drake simple car state messages to ignition
///        simple car state messages.
class DELPHYNE_BACKEND_VISIBLE
    DrakeSimpleCarStateToIgnSimpleCarStateTranslatorSystem
    : public DrakeToIgnTranslatorSystem<
          drake::automotive::SimpleCarState<double>,
          ignition::msgs::SimpleCarState> {
 public:
  DrakeSimpleCarStateToIgnSimpleCarStateTranslatorSystem();

 protected:
  // @brief @see DrakeToIgnTranslatorSystem::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(
      const drake::automotive::SimpleCarState<double>& drake_message,
      ignition::msgs::SimpleCarState* ign_message,
      int64_t time_ms) const override;
};

}  // namespace backend
}  // namespace delphyne
