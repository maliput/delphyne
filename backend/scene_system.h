// Copyright 2017 Toyota Research Institute

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <drake/systems/framework/leaf_system.h>

#include <ignition/msgs.hh>

#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that creates an ignition Scene message from a Model_V
/// message.
class SceneSystem
    : public drake::systems::LeafSystem<double> {
 public:
  SceneSystem();

 private:
  void CalcSceneMessage(const drake::systems::Context<double>& context,
                        ignition::msgs::Scene* scene_message) const;
};

}  // namespace backend
}  // namespace delphyne
