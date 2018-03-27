// Copyright 2017 Toyota Research Institute

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <drake/systems/framework/leaf_system.h>

#include "backend/system.h"

namespace delphyne {
namespace backend {

/// Publishes an ignition-transport message. The data to populate the message
/// comes from the system's abstract input port.
///
/// @tparam IGN_TYPE must be a valid ignition message type
template <class IGN_TYPE>
class IgnPublisherSystem : public drake::systems::LeafSystem<double> {
 public:
  /// Default constructor.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  /// be publishing to.
  /// @param[in] publish_period_ms The publishing period, in milliseconds. 0 to
  /// publish as fast as possible (on every simulation tick).
  explicit IgnPublisherSystem(const std::string& topic_name,
                              double publish_period_ms = 0)
      : topic_name_(topic_name), publish_period_(publish_period_ms) {
    DeclareAbstractInputPort();
    publisher_ = node_.Advertise<IGN_TYPE>(topic_name);
  }

  /// Default destructor.
  ~IgnPublisherSystem() override {}

  /// @see LeafSystem::DoPublish
  ///
  /// Takes an ignition message from the input port of the @p context and
  /// publishes it onto an ignition channel.
  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::PublishEvent<double>*>&)
      const override {
    // Conditional publishing checks and timekeeping are only performed if the
    // publishing period is non-zero (i.e. if this is a throttled publisher).
    if (publish_period_.count() > 0) {
      // Checks if it's time to publish.
      const std::chrono::time_point<std::chrono::steady_clock> now =
          std::chrono::steady_clock::now();
      const std::chrono::duration<double, std::milli> elapsed =
          now - last_publish_time_;
      if (elapsed < publish_period_) {
        return;
      }

      // It's time to publish!
      last_publish_time_ = now;
    }

    // Retrieves the ignition message content from input port.
    const int kPortIndex = 0;
    const drake::systems::AbstractValue* input =
        EvalAbstractInput(context, kPortIndex);
    DELPHYNE_DEMAND(input != nullptr);

    const IGN_TYPE& ign_message = input->GetValue<IGN_TYPE>();

    // Publishes onto the specified ign-transport topic.
    publisher_.Publish(ign_message);
  }

  /// Returns the topic name it publishes to.
  inline const std::string& get_topic_name() { return topic_name_; }

 private:
  // The topic on which to publish ign-transport messages.
  const std::string topic_name_;

  // Ignition transport node.
  ignition::transport::Node node_;

  // Ignition transport publisher.
  mutable ignition::transport::Node::Publisher publisher_;

  // The time between message publications (ms).
  const std::chrono::duration<double, std::milli> publish_period_;

  // The last time that a message was published at.
  mutable std::chrono::steady_clock::time_point last_publish_time_;
};

}  // namespace backend
}  // namespace delphyne
