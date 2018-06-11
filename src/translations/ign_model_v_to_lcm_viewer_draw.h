// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/lcmt_viewer_draw.hpp>

#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "translations/ign_to_drake.h"

namespace delphyne {

/// @brief A system that translates ignition Model_V messages to LCM viewer draw
/// messages.
class IgnModelVToLcmViewerDraw
    : public IgnToDrake<ignition::msgs::Model_V, drake::lcmt_viewer_draw> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::Model_V& ign_message,
      drake::lcmt_viewer_draw* lcm_message) const override;
};

}  // namespace delphyne
