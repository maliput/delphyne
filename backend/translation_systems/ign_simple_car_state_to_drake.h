// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/simple_car.h"

#include "ignition/msgs.hh"

#include "protobuf/simple_car_state.pb.h"

#include "backend/system.h"
#include "backend/translation_systems/ign_to_drake.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

/// @brief A system that translates ignition simple car state messages to Drake
/// simple car state messages.
class DELPHYNE_BACKEND_VISIBLE IgnSimpleCarStateToDrake
    : public IgnToDrake<ignition::msgs::SimpleCarState,
                        drake::automotive::SimpleCarState<double>> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::SimpleCarState& ign_message,
      drake::automotive::SimpleCarState<double>* drake_message) const override;
};

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne