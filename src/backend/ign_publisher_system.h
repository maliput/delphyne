// Copyright 2017 Toyota Research Institute

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <drake/systems/framework/leaf_system.h>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "delphyne/macros.h"

namespace delphyne {

/// A system to publish ignition messages at its single abstract input
/// port through an ignition transport topic.
///
/// @tparam IGN_TYPE A valid ignition message type.
template <typename IGN_TYPE,
          typename std::enable_if<
              std::is_base_of<ignition::transport::ProtoMsg, IGN_TYPE>::value,
              int>::type = 0>
class IgnPublisherSystem : public drake::systems::LeafSystem<double> {
 public:
  /// Constructs a publisher that forwards messages at a given fixed rate
  /// to the given ignition transport topic.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  ///                       be publishing to.
  /// @param[in] publish_rate The publishing rate, in Hz.
  /// @pre Given @p publish_rate is a positive number.
  /// @warning Failure to meet any of the preconditions will abort execution.
  explicit IgnPublisherSystem(const std::string& topic_name,
                              double publish_rate)
      : topic_name_(topic_name) {
    DELPHYNE_VALIDATE(publish_rate > 0.0, std::invalid_argument,
                      "Invalid publish rate (must be > 0.0)");
    this->DeclareAbstractInputPort(drake::systems::kUseDefaultName,
                                   drake::systems::Value<IGN_TYPE>());
    const double kPublishTimeOffset{0.};
    const drake::systems::PublishEvent<double> publish_event(
        drake::systems::Event<double>::TriggerType::kPeriodic,
        std::bind(&IgnPublisherSystem<IGN_TYPE>::PublishIgnMessage, this,
                  std::placeholders::_1, std::placeholders::_2));
    this->DeclarePeriodicEvent(1.0 / publish_rate, kPublishTimeOffset,
                               publish_event);
    publisher_ = node_.Advertise<IGN_TYPE>(topic_name);
  }

  /// Constructs a publisher that forwards messages at the fastest possible
  /// rate (i.e. on every simulation step) to the given ignition transport
  /// topic.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  ///                       be publishing to.
  explicit IgnPublisherSystem(const std::string& topic_name)
      : topic_name_(topic_name) {
    this->DeclareAbstractInputPort(drake::systems::kUseDefaultName,
                                   drake::systems::Value<IGN_TYPE>());
    const drake::systems::PublishEvent<double> publish_event(
        drake::systems::Event<double>::TriggerType::kPerStep,
        std::bind(&IgnPublisherSystem<IGN_TYPE>::PublishIgnMessage, this,
                  std::placeholders::_1, std::placeholders::_2));
    this->DeclarePerStepEvent(publish_event);
    publisher_ = node_.Advertise<IGN_TYPE>(topic_name);
  }

  /// Default destructor.
  ~IgnPublisherSystem() override {}

  /// Returns the topic name it publishes to.
  inline const std::string& get_topic_name() { return topic_name_; }

 private:
  // Takes the ignition message at the input port in the current
  // @p context and publishes it onto an ignition transport channel.
  // @see drake::systems::System::Publish, drake::systems::PublishEvent
  void PublishIgnMessage(const drake::systems::Context<double>& context,
                         const drake::systems::PublishEvent<double>& event) {
    // Retrieves the input value from the sole input port.
    const int kPortIndex = 0;
    // Publishes the message onto the specified
    // ignition transport topic.
    publisher_.Publish(
        *this->template EvalInputValue<IGN_TYPE>(context, kPortIndex));
  }

  // The topic on which to publish ignition transport messages.
  const std::string topic_name_;

  // Ignition transport node.
  ignition::transport::Node node_;

  // Ignition transport publisher.
  ignition::transport::Node::Publisher publisher_;
};

}  // namespace delphyne
