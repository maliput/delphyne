// Copyright 2017 Toyota Research Institute

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <drake/systems/framework/leaf_system.h>

#include <ignition/msgs.hh>

namespace delphyne {

/// @brief A system that creates an ignition Scene message from two Model_V
/// messages: one describing the geometry of the whole scene (which contains all
/// the models, including their Visual subfield), and one with the updated poses
/// for some of those models (the non-static ones). The full scene is created
/// by updating the poses of the geometry models, and it is placed on the system
/// output port.
class SceneSystem : public drake::systems::LeafSystem<double> {
 public:
  SceneSystem();

  // This port expects an ignition::msgs::Model_V message, containing all models
  // in the scene, including their geometry (ignition::msgs::Visual).
  const drake::systems::InputPort<double>& get_geometry_models_input_port() const {
    return get_input_port(geometry_models_input_port_index_);
  }

  int get_geometry_models_input_port_index() const { return geometry_models_input_port_index_; }

  // This port expects an ignition::msgs::Model_V message containing the
  // up-to-date pose for all mobile elements (static scene elements do not
  // require a pose update).
  const drake::systems::InputPort<double>& get_updated_pose_models_input_port() const {
    return get_input_port(updated_pose_models_input_port_index_);
  }

  int get_updated_pose_models_input_port_index() const { return updated_pose_models_input_port_index_; }

 private:
  // Calculates a new scene message from the geometry description and the
  // updated poses of mobile elements.
  void CalcSceneMessage(const drake::systems::Context<double>& context, ignition::msgs::Scene* scene_message) const;

  int geometry_models_input_port_index_{};
  int updated_pose_models_input_port_index_{};
};

}  // namespace delphyne
