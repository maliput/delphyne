// Copyright 2018 Toyota Research Institute

#pragma once

#include <cstdint>

#include <drake/lcmt_viewer_draw.hpp>

#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

/// @brief A system that translates LCM viewer draw messages to ignition
/// Model_V.
class LcmViewerDrawToIgnModelV : public DrakeToIgn<drake::lcmt_viewer_draw, ignition::msgs::Model_V> {
 protected:
  // @brief @see DrakeToIgn::DoLcmToIgnTranslation.
  void DoDrakeToIgnTranslation(const drake::lcmt_viewer_draw& lcm_message, ignition::msgs::Model_V* ign_message,
                               int64_t time) const override;
};

}  // namespace delphyne
