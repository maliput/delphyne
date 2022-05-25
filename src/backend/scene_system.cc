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

#include "backend/scene_system.h"

#include <algorithm>

#include <ignition/common/Console.hh>

#include "delphyne/macros.h"

namespace delphyne {

template <class T>
using ProtobufIterator = google::protobuf::internal::RepeatedPtrIterator<T>;

const ignition::math::Color SceneSystem::kLightColor{0.9, 0.9, 0.9};

const ignition::math::Vector3d SceneSystem::kLightDirection{-0.5, -0.5, -1};

SceneSystem::SceneSystem() {
  geometry_models_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<ignition::msgs::Model_V>()).get_index();
  updated_pose_models_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<ignition::msgs::Model_V>()).get_index();
  updated_visual_models_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<ignition::msgs::Model_V>()).get_index();

  DeclareAbstractOutputPort(&SceneSystem::CalcSceneMessage);
}

void SceneSystem::CalcSceneMessage(const drake::systems::Context<double>& context,
                                   ignition::msgs::Scene* scene_message) const {
  DELPHYNE_VALIDATE(scene_message != nullptr, std::invalid_argument, "Scene message pointer must not be null");

  // Clears old scene state from the previous CalcSceneMessage call.
  // @see DeclareAbstractOutputPort
  scene_message->Clear();

  const ignition::msgs::Model_V* geometry_models =
      this->template EvalInputValue<ignition::msgs::Model_V>(context, geometry_models_input_port_index_);

  const ignition::msgs::Model_V* updated_pose_models =
      this->template EvalInputValue<ignition::msgs::Model_V>(context, updated_pose_models_input_port_index_);

  const ignition::msgs::Model_V* updated_visual_models =
      this->template EvalInputValue<ignition::msgs::Model_V>(context, updated_visual_models_input_port_index_);

  // The scene timestamp is that of the poses update.
  scene_message->mutable_header()->mutable_stamp()->CopyFrom(updated_pose_models->header().stamp());

  // All of the geometry models are added to the scene.
  for (const ignition::msgs::Model& geometry_model : geometry_models->models()) {
    scene_message->add_model()->CopyFrom(geometry_model);
  }

  // And those models for which a pose update exists are then updated.
  for (const ignition::msgs::Model& updated_pose_model : updated_pose_models->models()) {
    // Finds the matching scene model.
    const ProtobufIterator<ignition::msgs::Model>& matching_scene_model =
        std::find_if(scene_message->mutable_model()->begin(), scene_message->mutable_model()->end(),
                     [&updated_pose_model](const ::ignition::msgs::Model& scene_model) {
                       return scene_model.id() == updated_pose_model.id();
                     });

    if (matching_scene_model == scene_message->mutable_model()->end()) {
      ignerr << "No geometry model for updated pose with model id " << updated_pose_model.id() << std::endl;
      continue;
    }

    // Updates the model itself.
    matching_scene_model->set_name(updated_pose_model.name());
    matching_scene_model->mutable_pose()->CopyFrom(updated_pose_model.pose());

    // Updates each model link.
    for (const ignition::msgs::Link& updated_pose_link : updated_pose_model.link()) {
      // Finds the corresponding link inside the scene model.
      const ProtobufIterator<ignition::msgs::Link>& matching_scene_link =
          std::find_if(matching_scene_model->mutable_link()->begin(), matching_scene_model->mutable_link()->end(),
                       [&updated_pose_link](const ::ignition::msgs::Link& scene_link) {
                         return scene_link.name() == updated_pose_link.name();
                       });

      if (matching_scene_link == matching_scene_model->mutable_link()->end()) {
        ignerr << "No geometry link for updated pose with model id " << updated_pose_model.id() << " and link name "
               << updated_pose_link.name() << std::endl;
        continue;
      }

      // Applies the pose to the scene link.
      matching_scene_link->mutable_pose()->mutable_position()->CopyFrom(updated_pose_link.pose().position());
      matching_scene_link->mutable_pose()->mutable_orientation()->CopyFrom(updated_pose_link.pose().orientation());
    }
  }

  if (updated_visual_models != nullptr) {
    // And those models for which a material update exists are then updated.
    for (const ignition::msgs::Model& updated_visual_model : updated_visual_models->models()) {
      // Finds the matching scene model.
      const ProtobufIterator<ignition::msgs::Model>& matching_scene_model =
          std::find_if(scene_message->mutable_model()->begin(), scene_message->mutable_model()->end(),
                       [&updated_visual_model](const ::ignition::msgs::Model& scene_model) {
                         return scene_model.id() == updated_visual_model.id();
                       });

      if (matching_scene_model == scene_message->mutable_model()->end()) {
        ignerr << "No geometry model for updated visual with model id " << updated_visual_model.id() << std::endl;
        continue;
      }
      // Updates the model itself.
      matching_scene_model->set_name(updated_visual_model.name());

      // Updates each model link.
      for (const ignition::msgs::Link& updated_light_link : updated_visual_model.link()) {
        // Finds the corresponding link inside the scene model.
        const ProtobufIterator<ignition::msgs::Link>& matching_scene_link =
            std::find_if(matching_scene_model->mutable_link()->begin(), matching_scene_model->mutable_link()->end(),
                         [&updated_light_link](const ::ignition::msgs::Link& scene_link) {
                           return scene_link.name() == updated_light_link.name();
                         });

        if (matching_scene_link == matching_scene_model->mutable_link()->end()) {
          ignerr << "No geometry link for updated visual with model id " << updated_visual_model.id()
                 << " and link name " << updated_light_link.name() << std::endl;
          continue;
        }

        // Updates the visual of the link.
        matching_scene_link->mutable_visual()->CopyFrom(updated_light_link.visual());
      }
    }
  }

  // Add a directional light to the scene
  {
    const ignition::msgs::Color kLightColorMsg = ignition::msgs::Convert(kLightColor);
    ignition::msgs::Light directionalLight;
    directionalLight.set_name("directional_light");
    directionalLight.set_type(ignition::msgs::Light_LightType_DIRECTIONAL);
    directionalLight.mutable_diffuse()->CopyFrom(kLightColorMsg);
    directionalLight.mutable_specular()->CopyFrom(kLightColorMsg);
    directionalLight.mutable_direction()->CopyFrom(ignition::msgs::Convert(kLightDirection));
    directionalLight.set_cast_shadows(kCastShadowsByDefault);
    scene_message->add_light()->CopyFrom(directionalLight);
  }
}

}  // namespace delphyne
