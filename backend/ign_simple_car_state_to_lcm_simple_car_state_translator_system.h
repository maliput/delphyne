// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/automotive/simple_car.h"

#include "ignition/msgs.hh"

#include "protobuf/simple_car_state.pb.h"

#include "backend/ign_to_lcm_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates ignition simple car state messages to LCM
/// simple car state messages.
class DELPHYNE_BACKEND_VISIBLE
    IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystem
    : public IgnToLcmTranslatorSystem<
          ignition::msgs::SimpleCarState,
          drake::automotive::SimpleCarState<double>> {
 public:
  /// @brief Default constructor. @see IgnToLcmTranslatorSystem::InitPorts.
  IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystem();

 protected:
  // @brief @see IgnToLcmTranslatorSystem::DoIgnToLcmTranslation.
  void DoIgnToLcmTranslation(
      const ignition::msgs::SimpleCarState& ign_message,
      drake::automotive::SimpleCarState<double>* lcm_message) const override;
};

}  // namespace backend
}  // namespace delphyne
