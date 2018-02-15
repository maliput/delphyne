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

#include <chrono>
#include <string>
#include <vector>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <drake/lcmt_viewer_draw.hpp>
#include <drake/systems/framework/leaf_system.h>

#include "backend/system.h"
#include "bridge/lcm_to_ign_translation.h"

namespace delphyne {
namespace backend {

/// Publishes an ignition-transport message containing
/// information about the scene.
class DELPHYNE_BACKEND_VISIBLE SceneSystem
    : public drake::systems::LeafSystem<double> {
 public:
  // Class constructor. Accepts the topic name that will be used to publish
  // the scene message.
  explicit SceneSystem(const std::string& topic_name);

  // Class destructor.
  ~SceneSystem() override;

  /// Takes the VectorBase from the input port of the `context`
  /// and publishes it onto an ignition-transport topic.
  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::PublishEvent<double>*>&)
      const override;

  /// \brief Get the ign-transport topic name.
  /// \return Ignition transport topic name.
  inline const std::string& get_topic_name() { return topic_; }

 private:
  // The period between scene updates (ms).
  const double kScenePeriodMs_ = 250.0;
  // The topic on which to publish ign-transport messages.
  const std::string topic_;
  // Ignition transport node.
  ignition::transport::Node node_;
  // Ignition transport publisher.
  mutable ignition::transport::Node::Publisher publisher_;
  // The last time that the scene message was updated.
  mutable std::chrono::steady_clock::time_point last_scene_update_;
};

}  // namespace backend
}  // namespace delphyne
