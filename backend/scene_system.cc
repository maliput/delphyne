// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"

#include <algorithm>

#include <ignition/common/Console.hh>

namespace delphyne {
namespace backend {

SceneSystem::SceneSystem() {
  geometry_models_input_port_index_ = DeclareAbstractInputPort().get_index();
  updated_pose_models_input_port_index_ =
      DeclareAbstractInputPort().get_index();

  DeclareAbstractOutputPort(&SceneSystem::CalcSceneMessage);
}

const drake::systems::InputPortDescriptor<double>&
SceneSystem::get_geometry_models_input_port() const {
  return get_input_port(geometry_models_input_port_index_);
}

const drake::systems::InputPortDescriptor<double>&
SceneSystem::get_updated_pose_models_input_port() const {
  return get_input_port(updated_pose_models_input_port_index_);
}

void SceneSystem::CalcSceneMessage(
    const drake::systems::Context<double>& context,
    ignition::msgs::Scene* scene_message) const {
  DELPHYNE_DEMAND(scene_message != nullptr);

  // Clears old scene state from the previous CalcSceneMessage call.
  // @see DeclareAbstractOutputPort
  scene_message->Clear();

  const drake::systems::AbstractValue* geometry_input =
      EvalAbstractInput(context, geometry_models_input_port_index_);
  const auto& geometry_models =
      geometry_input->GetValue<ignition::msgs::Model_V>();

  const drake::systems::AbstractValue* updated_pose_input =
      EvalAbstractInput(context, updated_pose_models_input_port_index_);
  const auto& updated_pose_models =
      updated_pose_input->GetValue<ignition::msgs::Model_V>();

  // The scene is then built from the geometry models, which are updated using
  // the updated poses.
  for (const ignition::msgs::Model& geometry_model : geometry_models.models()) {
    ignition::msgs::Model* new_model = scene_message->add_model();
    new_model->CopyFrom(geometry_model);

    // Finds the updated pose model for the model.
    const google::protobuf::internal::RepeatedPtrIterator<
        const ignition::msgs::Model>& matching_updated_pose_model =
        std::find_if(updated_pose_models.models().begin(),
                     updated_pose_models.models().end(),
                     [geometry_model](
                         const ::ignition::msgs::Model& updated_pose_model) {
                       return updated_pose_model.id() == geometry_model.id();
                     });

    if (matching_updated_pose_model == updated_pose_models.models().end()) {
      ignerr << "No updated pose for model id " << geometry_model.id()
             << std::endl;
      continue;
    }

    // Updates each model link.
    for (const ignition::msgs::Link& geometry_link : geometry_model.link()) {
      // Find the corresponding link inside the updated pose model.
      const google::protobuf::internal::RepeatedPtrIterator<
          const ignition::msgs::Link>& matching_updated_pose_link =
          std::find_if(
              matching_updated_pose_model->link().begin(),
              matching_updated_pose_model->link().end(),
              [geometry_link](const ::ignition::msgs::Link& updated_pose_link) {
                return updated_pose_link.name() == geometry_link.name();
              });

      if (matching_updated_pose_link ==
          matching_updated_pose_model->link().end()) {
        ignerr << "No updated pose for link name " << geometry_link.name()
               << std::endl;
        continue;
      }

      // Gets the pose and applies it to the model
      new_model->mutable_pose()->mutable_position()->CopyFrom(
          matching_updated_pose_link->pose().position());
      new_model->mutable_pose()->mutable_orientation()->CopyFrom(
          matching_updated_pose_link->pose().orientation());
    }
  }

  // The scene timestamp is that of the poses update.
  scene_message->mutable_header()->mutable_stamp()->CopyFrom(
      updated_pose_models.header().stamp());

  // TODO(caguero): Populate the rest of the scene fields, such as lights.
  // See https://github.com/ToyotaResearchInstitute/delphyne/issues/204
}

}  // namespace backend
}  // namespace delphyne
