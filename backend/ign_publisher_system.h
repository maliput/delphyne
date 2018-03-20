// Copyright 2017 Toyota Research Institute

#pragma once

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
  explicit IgnPublisherSystem(const std::string& topic_name)
      : topic_name_(topic_name) {
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
};

}  // namespace backend
}  // namespace delphyne
