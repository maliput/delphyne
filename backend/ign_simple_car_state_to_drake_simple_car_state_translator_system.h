// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/simple_car.h"

#include "ignition/msgs.hh"

#include "protobuf/simple_car_state.pb.h"

#include "backend/ign_to_drake_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates ignition simple car state messages to Drake
/// simple car state messages.
class DELPHYNE_BACKEND_VISIBLE
    IgnSimpleCarStateToDrakeSimpleCarStateTranslatorSystem
    : public IgnToDrakeTranslatorSystem<
          ignition::msgs::SimpleCarState,
          drake::automotive::SimpleCarState<double>> {
 protected:
  // @brief @see IgnToDrakeTranslatorSystem::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::SimpleCarState& ign_message,
      drake::automotive::SimpleCarState<double>* drake_message) const override;
};

}  // namespace backend
}  // namespace delphyne
