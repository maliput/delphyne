// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <drake/systems/framework/leaf_system.h>

#include "backend/discrete_value_to_ignition_message_converter.h"
#include "backend/ignition_message_converter.h"
#include "backend/system.h"

using drake::systems::Context;
using drake::systems::PublishEvent;

namespace delphyne {
namespace backend {

/// Publishes an ignition-transport message. The data to populate the message
/// comes from the system's input port and is passed to a converter, which
/// is in charge of processing the input port information and retrieve an
/// ignition message.
///
/// @tparam IGN_TYPE must be a valid ignition message type
template <class IGN_TYPE>
class IgnPublisherSystem : public drake::systems::LeafSystem<double> {
 public:
  /// Default constructor.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  /// be publishing to.
  ///
  /// @param[in] converter The object in charge of populating an ignition
  /// message content from the data in the input port.
  explicit IgnPublisherSystem(
      const std::string& topic_name,
      std::unique_ptr<IgnitionMessageConverter<IGN_TYPE>> converter)
      : topic_name_(topic_name), converter_(std::move(converter)) {
    DELPHYNE_DEMAND(converter_ != nullptr);

    if (converter_->handles_discrete_values()) {
      this->DeclareInputPort(drake::systems::kVectorValued,
                             converter_->get_vector_size());
    } else {
      this->DeclareAbstractInputPort();
    }
    publisher_ = node_.Advertise<IGN_TYPE>(topic_name);
  }

  /// Default destructor.
  ~IgnPublisherSystem() override {}

  /// @see LeafSystem::DoPublish
  ///
  /// Takes the data from the input port of the @p context and publishes
  /// it onto an ignition channel, using the converter to populate the
  /// ignition message.
  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::PublishEvent<double>*>&)
      const override {
    const int kPortIndex = 0;

    IGN_TYPE ign_msg;

    // Fill the ignition message content from input port.
    converter_->ProcessInput(this, context, kPortIndex, &ign_msg);

    // Publishes onto the specified ign-transport topic.
    publisher_.Publish(ign_msg);
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

  // Converts input port values to ignition messages.
  mutable std::unique_ptr<IgnitionMessageConverter<IGN_TYPE>> converter_{};
};

}  // namespace backend
}  // namespace delphyne
