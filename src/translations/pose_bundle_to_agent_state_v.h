// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/systems/rendering/pose_bundle.h>

#include <ignition/msgs.hh>

#include "delphyne/protobuf/agent_state_v.pb.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

/// @brief A system that takes a PoseBundle and generates an array of
/// AgentState (AgentState_V).
class PoseBundleToAgentState_V
    : public DrakeToIgn<drake::systems::rendering::PoseBundle<double>, ignition::msgs::AgentState_V> {
 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(const drake::systems::rendering::PoseBundle<double>& drake_message,
                               ignition::msgs::AgentState_V* ign_message, int64_t time_ms) const override;
};

}  // namespace delphyne
