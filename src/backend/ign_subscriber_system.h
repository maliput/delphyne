// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <drake/lcmt_viewer_draw.hpp>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/automotive_driving_command.pb.h"
#include "gen/driving_command.h"

namespace delphyne {

/// This class is the counterpart of Drake's LcmSubscriberSystem. Most of the
/// code has been taken from that class and adapted to the types of values
/// that we use.
///
/// @tparam IGN_TYPE must be a valid ignition message type.
template <class IGN_TYPE>
class IgnSubscriberSystem : public drake::systems::LeafSystem<double> {
 public:
  /// Default constructor.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  /// be subscribed to.
  explicit IgnSubscriberSystem(const std::string& topic_name)
      : topic_name_(topic_name) {
    DeclareAbstractOutputPort(
        [this]() { return this->AllocateDefaultAbstractValue(); },
        [this](const drake::systems::Context<double>& context,
               drake::AbstractValue* out) {
          this->IgnSubscriberSystem::CalcIgnMessage(context, out);
        });

    if (!node_.Subscribe(topic_name_,
                         &IgnSubscriberSystem<IGN_TYPE>::HandleMessage, this)) {
      ignerr << "Error subscribing to topic: " << topic_name_
             << "\n Ignition Subscriber will not work" << std::endl;
    }
  }

  ~IgnSubscriberSystem() override {}

  std::unique_ptr<drake::AbstractValue> AllocateDefaultAbstractValue()
      const {
    return std::make_unique<drake::Value<IGN_TYPE>>(IGN_TYPE{});
  }

  std::unique_ptr<drake::systems::AbstractValues> AllocateAbstractState()
      const override {
    std::vector<std::unique_ptr<drake::AbstractValue>> abstract_values(
        kTotalStateValues);
    abstract_values[kStateIndexMessage] = AllocateDefaultAbstractValue();
    abstract_values[kStateIndexMessageCount] =
        drake::AbstractValue::Make<int>(0);
    return std::make_unique<drake::systems::AbstractValues>(
        std::move(abstract_values));
  }

  void SetDefaultState(const drake::systems::Context<double>&,
                       drake::systems::State<double>* state) const override {
    DELPHYNE_VALIDATE(state != nullptr, std::invalid_argument,
                      "State pointer must not be null");
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

  /// Returns the topic name it is subscribed to.
  inline const std::string& get_topic_name() { return topic_name_; }

  /// Returns the message counter stored in @p context.
  int GetMessageCount(const drake::systems::Context<double>& context) const {
    return context.get_abstract_state()
        .get_value(kStateIndexMessageCount)
        .GetValue<int>();
  }

 protected:
  // Callback invoked each time a new message is received in the ignition
  // topic we are subscribed to. Sets it as the last received message and
  // increases the received message count
  //
  // @param[in] ignition_message The message received.
  void HandleMessage(const IGN_TYPE& ignition_message) {
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    last_received_message_ = ignition_message;
    received_message_count_++;
  }

  void DoCalcNextUpdateTime(
      const drake::systems::Context<double>& context,
      drake::systems::CompositeEventCollection<double>* events,
      double* time) const override {
    DELPHYNE_VALIDATE(events != nullptr, std::invalid_argument,
                      "Events pointer must not be null");
    DELPHYNE_VALIDATE(time != nullptr, std::invalid_argument,
                      "Time pointer must not be null");

    // An update time calculation is required here to avoid having
    // a NaN value when callling the StepBy method.
    drake::systems::LeafSystem<double>::DoCalcNextUpdateTime(context, events,
                                                             time);

    const int last_message_count = GetMessageCount(context);

    const int received_message_count = [this]() {
      std::unique_lock<std::mutex> lock(received_message_mutex_);
      return received_message_count_;
    }();

    // Has a new message. Schedule an update event.
    if (last_message_count != received_message_count) {
      // From Drake LCM implementation:
      // TODO(siyuan): should be context.get_time() once #5725 is resolved.
      *time = context.get_time() + 0.0001;

      drake::systems::EventCollection<
          drake::systems::UnrestrictedUpdateEvent<double>>& uu_events =
          events->get_mutable_unrestricted_update_events();

      uu_events.add_event(
          std::make_unique<drake::systems::UnrestrictedUpdateEvent<double>>(
              drake::systems::Event<double>::TriggerType::kTimed));
    }
  }

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<double>&,
      const std::vector<
          const drake::systems::UnrestrictedUpdateEvent<double>*>&,
      drake::systems::State<double>* state) const override {
    DELPHYNE_VALIDATE(state != nullptr, std::invalid_argument,
                      "State pointer must not be null");

    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

  void ProcessMessageAndStoreToAbstractState(
      drake::systems::AbstractValues* abstract_state) const {
    DELPHYNE_VALIDATE(abstract_state != nullptr, std::invalid_argument,
                      "Abstract state pointer must not be null");

    std::lock_guard<std::mutex> lock(received_message_mutex_);

    if (received_message_count_ > 0) {
      abstract_state->get_mutable_value(kStateIndexMessage)
          .GetMutableValue<IGN_TYPE>() = last_received_message_;
    }

    abstract_state->get_mutable_value(kStateIndexMessageCount)
        .GetMutableValue<int>() = received_message_count_;
  }

  void CalcIgnMessage(const drake::systems::Context<double>& context,
                      drake::AbstractValue* output_value) const {
    DELPHYNE_VALIDATE(output_value != nullptr, std::invalid_argument,
                      "Output value pointer must not be null");

    output_value->SetFrom(
        context.get_abstract_state().get_value(kStateIndexMessage));
  }

 private:
  // The topic on which to publish ign-transport messages.
  const std::string topic_name_;

  // The mutex that guards last_received_message_ and received_message_count_.
  mutable std::mutex received_message_mutex_;

  // The bytes of the most recently received LCM message.
  IGN_TYPE last_received_message_;

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  // Ignition transport node.
  // The ignition transport node must be declared after everything its callbacks
  // use.  This ensures that it is constructed after everything it needs is
  // properly setup, and destroyed before everything it needs is destroyed,
  // avoiding a race where the callback for a subscribed topic can be called
  // after the member variables it needs have already been destroyed.
  ignition::transport::Node node_;

  // The index of the state used to access the the message.
  static constexpr int kStateIndexMessage = 0;

  // The index of the state used to access the the message count.
  static constexpr int kStateIndexMessageCount = 1;

  // The total number of states used by the system.
  static constexpr int kTotalStateValues = 2;
};

}  // namespace delphyne
