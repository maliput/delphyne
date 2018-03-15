// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/lcmt_viewer_draw.hpp"

#include "ignition/msgs.hh"

#include "backend/ign_to_drake_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates ignition Model V messages to LCM viewer draw
///        messages.
class DELPHYNE_BACKEND_VISIBLE IgnModelVToLcmViewerDrawTranslatorSystem
    : public IgnToDrakeTranslatorSystem<ignition::msgs::Model_V,
                                        drake::lcmt_viewer_draw> {
 protected:
  // @brief @see IgnToDrakeTranslatorSystem::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(
      const ignition::msgs::Model_V& ign_message,
      drake::lcmt_viewer_draw* lcm_message) const override;
};

}  // namespace backend
}  // namespace delphyne
