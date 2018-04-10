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

  const drake::systems::InputPortDescriptor<double>&
  get_geometry_models_input_port() const {
    return get_input_port(geometry_models_input_port_index);
  }

  const drake::systems::InputPortDescriptor<double>&
  get_updated_pose_models_input_port() const {
    return get_input_port(updated_pose_models_input_port_index);
  }

  void GetScene(ignition::msgs::Scene* scene) const;

  int geometry_models_input_port_index;
  int updated_pose_models_input_port_index;

 private:
  void CalcSceneMessage(const drake::systems::Context<double>& context,
                        ignition::msgs::Scene* scene_message) const;
};

}  // namespace backend
}  // namespace delphyne
