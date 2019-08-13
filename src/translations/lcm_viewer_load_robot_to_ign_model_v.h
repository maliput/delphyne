// Copyright 2018 Toyota Research Institute

#pragma once

#include <cstdint>

#include <drake/lcmt_viewer_load_robot.hpp>

#include <ignition/msgs.hh>

#include "translations/drake_to_ign.h"

namespace delphyne {

/// @brief A system that translates LCM viewer load robot messages to ignition
/// Model_V.
class LcmViewerLoadRobotToIgnModelV : public DrakeToIgn<drake::lcmt_viewer_load_robot, ignition::msgs::Model_V> {
 protected:
  // @brief @see DrakeToIgn::DoLcmToIgnTranslation.
  void DoDrakeToIgnTranslation(const drake::lcmt_viewer_load_robot& lcm_message, ignition::msgs::Model_V* ign_message,
                               int64_t time) const override;
};

}  // namespace delphyne
