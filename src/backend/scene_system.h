// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <drake/systems/framework/leaf_system.h>
#include <ignition/math/Color.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs.hh>

namespace delphyne {

/// @brief A system that creates an ignition Scene message from three Model_V
/// messages: one describing the geometry of the whole scene (which contains all
/// the models, including their Visual subfield), another one with the updated poses
/// for some of those models (the non-static ones) and the third one containing the updated
/// visuals for some of those models. The full scene is then created and placed on the system output port.
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

  int get_updated_visual_models_input_port_index() const { return updated_visual_models_input_port_index_; }

  const drake::systems::InputPort<double>& get_updated_visual_models_input_port() const {
    return get_input_port(updated_visual_models_input_port_index_);
  }

 private:
  // Calculates a new scene message from the geometry description and the
  // updated poses of mobile elements.
  void CalcSceneMessage(const drake::systems::Context<double>& context, ignition::msgs::Scene* scene_message) const;

  // This is the color used by the directional light added to each scene.
  static const ignition::math::Color kLightColor;

  // This is the direction of the directional light added to each scene.
  static const ignition::math::Vector3d kLightDirection;

  // Cast shadows by default.
  static const bool kCastShadowsByDefault{true};

  int geometry_models_input_port_index_{};
  int updated_pose_models_input_port_index_{};
  int updated_visual_models_input_port_index_{};
};

}  // namespace delphyne
