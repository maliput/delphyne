// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/lcmt_viewer_draw.hpp"

#include "ignition/msgs.hh"

#include "backend/lcm_to_ign_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates LCM viewer draw messages to ignition Model
/// V.
class DELPHYNE_BACKEND_VISIBLE LcmViewerDrawToIgnModelVTranslatorSystem
    : public LcmToIgnTranslatorSystem<drake::lcmt_viewer_draw,
                                      ignition::msgs::Model_V> {
 public:
  /// @brief Default constructor. @see LcmToIgnTranslatorSystem::InitPorts.
  LcmViewerDrawToIgnModelVTranslatorSystem();

 protected:
  // @brief @see LcmToIgnTranslatorSystem::DoLcmToIgnTranslation.
  void DoLcmToIgnTranslation(const drake::lcmt_viewer_draw& lcm_message,
                             ignition::msgs::Model_V* ign_message,
                             int64_t time) const override;
};

}  // namespace backend
}  // namespace delphyne
