// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"

#include <ignition/msgs.hh>

#include "backend/system.h"
#include "backend/lcm_to_ign_translation.h"

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
