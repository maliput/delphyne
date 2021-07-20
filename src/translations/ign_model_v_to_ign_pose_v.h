// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/lcmt_viewer_draw.hpp>
#include <ignition/msgs.hh>

#include "delphyne/macros.h"
#include "translations/ign_to_drake.h"

namespace delphyne {

/// @brief A system that translates ignition Model_V messages to ignition Pose_V messages
/// messages.
class IgnModelVToIgnPoseV : public IgnToDrake<ignition::msgs::Model_V, ignition::msgs::Pose_V> {
 protected:
  // @brief @see IgnToDrake::DoIgnToDrakeTranslation.
  void DoIgnToDrakeTranslation(const ignition::msgs::Model_V& ign_model_v,
                               ignition::msgs::Pose_V* ign_pose_v) const override;
};

}  // namespace delphyne
