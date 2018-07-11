// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/systems/rendering/pose_bundle.h>

#include <ignition/msgs.hh>

#include "delphyne/protobuf/simple_car_state_v.pb.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

/// @brief A system that takes a PoseBundle and generates an array of
/// SimpleCarStates (SimpleCarState_V).
class PoseBundleToSimpleCarState_V
    : public DrakeToIgn<drake::systems::rendering::PoseBundle<double>,
                        ignition::msgs::SimpleCarState_V> {
 protected:
  // @brief @see DrakeToIgn::DoDrakeToIgnTranslation.
  void DoDrakeToIgnTranslation(
      const drake::systems::rendering::PoseBundle<double>& drake_message,
      ignition::msgs::SimpleCarState_V* ign_message,
      int64_t time_ms) const override;
};

}  // namespace delphyne
