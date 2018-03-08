// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"

#include <ignition/msgs.hh>

#include "backend/lcm_to_ign_translation.h"
#include "backend/system.h"

using drake::systems::rendering::PoseBundle;
using drake::systems::Context;

namespace delphyne {
namespace backend {

/// This system is in charge of caching the last update of the models' poses
/// in a simulation and apply those poses to a model centered in the origin.
/// TODO(basicNew): In the future we will move more code from the bridge into
/// this class and create the model from scratch instead of updating an
/// existing one. This process should include merging the work done in
/// SceneSystem in this class. See
/// https://github.com/ToyotaResearchInstitute/delphyne/issues/218
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// They are already available to link against in the containing library.
template <typename T>
class DELPHYNE_BACKEND_VISIBLE SceneBuilderSystem
    : public drake::systems::LeafSystem<T> {
 public:
  SceneBuilderSystem();

  /// @see LeafSystem::DoPublish
  ///
  /// Takes the data from the input port of the @p context and stores
  /// a copy of it, keeping a cache of the last pose bundle update.
  void DoPublish(const drake::systems::Context<T>& context,
                 const std::vector<const drake::systems::PublishEvent<T>*>&)
      const override;

  /// Given a collection of models, update their poses based on the cached
  /// pose bundle this class holds
  ///
  /// @param[out] robot_models The models to be updated with the new poses
  void UpdateModels(ignition::msgs::Model_V* robot_models);

 private:
  // Caches the last pose bundle received in the input port.
  mutable std::unique_ptr<PoseBundle<T>> last_poses_update_;
};

}  // namespace backend
}  // namespace delphyne
