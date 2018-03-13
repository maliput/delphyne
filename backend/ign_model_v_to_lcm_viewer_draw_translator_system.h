// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/lcmt_viewer_draw.hpp"

#include "ignition/msgs.hh"

#include "backend/ign_to_lcm_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates ignition Model V messages to LCM viewer
///        draw.
class DELPHYNE_BACKEND_VISIBLE IgnModelVToLcmViewerDrawTranslatorSystem
    : public IgnToLcmTranslatorSystem<ignition::msgs::Model_V,
                                      drake::lcmt_viewer_draw> {
 public:
  /// @brief Default constructor. @see IgnToLcmTranslatorSystem::InitPorts.
  IgnModelVToLcmViewerDrawTranslatorSystem();

 protected:
  // @brief @see IgnToLcmTranslatorSystem::DoIgnToLcmTranslation.
  void DoIgnToLcmTranslation(
      const ignition::msgs::Model_V& ign_message,
      drake::lcmt_viewer_draw* lcm_message) const override;
};

}  // namespace backend
}  // namespace delphyne
