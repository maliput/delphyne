// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/lcmt_viewer_load_robot.hpp"

#include "ignition/msgs.hh"

#include "backend/translation_systems/drake_to_ign.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

/// @brief A system that translates LCM viewer load robot messages to ignition
/// Model_V.
class DELPHYNE_BACKEND_VISIBLE LcmViewerLoadRobotToIgnModelV
    : public DrakeToIgn<drake::lcmt_viewer_load_robot,
                        ignition::msgs::Model_V> {
 protected:
  // @brief @see DrakeToIgn::DoLcmToIgnTranslation.
  void DoDrakeToIgnTranslation(const drake::lcmt_viewer_load_robot& lcm_message,
                               ignition::msgs::Model_V* ign_message,
                               int64_t time) const override;
};

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
