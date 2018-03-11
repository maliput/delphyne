// Copyright 2017 Toyota Research Institute

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <drake/lcmt_viewer_draw.hpp>
#include <drake/systems/framework/leaf_system.h>

#include "backend/lcm_to_ign_translation.h"
#include "backend/system.h"

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
