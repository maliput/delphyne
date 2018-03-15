// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/lcmt_viewer_draw.hpp"

#include "ignition/msgs.hh"

#include "backend/system.h"
#include "backend/translation_systems/ign_to_drake.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

/// @brief A system that translates ignition Model V messages to LCM viewer draw
/// messages.
class DELPHYNE_BACKEND_VISIBLE IgnModelVToLcmViewerDraw
    : public IgnToDrake<ignition::msgs::Model_V, drake::lcmt_viewer_draw> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::Model_V& ign_message,
      drake::lcmt_viewer_draw* lcm_message) const override;
};

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
