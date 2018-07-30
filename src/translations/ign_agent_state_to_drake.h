// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/automotive/simple_car.h>

#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/agent_state.pb.h"
#include "translations/ign_to_drake.h"

namespace delphyne {

/// @brief A system that translates ignition simple car state messages to Drake
/// simple car state messages.
class IgnAgentStateToDrake
    : public IgnToDrake<ignition::msgs::AgentState,
                        drake::automotive::SimpleCarState<double>> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::AgentState& ign_message,
      drake::automotive::SimpleCarState<double>* drake_message) const override;
};

}  // namespace delphyne
