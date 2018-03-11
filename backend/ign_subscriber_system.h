// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <drake/automotive/gen/driving_command.h>
#include <drake/lcmt_viewer_draw.hpp>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include <protobuf/automotive_driving_command.pb.h>

#include "backend/ignition_message_converter.h"
#include "backend/system.h"

using drake::systems::AbstractValue;
using drake::systems::AbstractValues;
using drake::systems::DiscreteValues;
using drake::systems::Context;
using drake::systems::State;
using drake::systems::BasicVector;
using drake::systems::LeafSystem;

namespace delphyne {
namespace backend {

/// This class is the counterpart of Drake's LcmSubscriberSystem. Most of the
/// code has been taken from that class and adapted to the types of values
/// that we use (e.g. we don't make the distinction between having a
/// serializer and a translator, we just have a converter).
///
/// @tparam IGN_TYPE must be a valid ignition message type.
template <class IGN_TYPE>
class IgnSubscriberSystem : public LeafSystem<double> {
 public:
  /// Default constructor.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  /// be subscribed to.
  ///
  /// @param[in] converter The object that will be used to convert between the
  /// ignition messages we receive on the topic and what we send through the
  /// output port.
  explicit IgnSubscriberSystem(
      const std::string& topic_name,
      std::unique_ptr<IgnitionMessageConverter<IGN_TYPE>> converter)
      : topic_name_(topic_name), converter_(std::move(converter)) {
    DELPHYNE_DEMAND(converter_ != nullptr);

    if (converter_->handles_discrete_values()) {
      DeclareVectorOutputPort(
          *AllocateDiscreteOutputValue(),
          &IgnSubscriberSystem<IGN_TYPE>::CalcDiscreteOutputValue);
    } else {
      DeclareAbstractOutputPort(
          [this](const Context<double>&) {
            return this->AllocateAbstractOutputValue();
          },
          [this](const Context<double>& context, AbstractValue* out) {
            this->CalcAbstractOutputValue(context, out);
          });
    }

    if (!node_.Subscribe(topic_name_,
                         &IgnSubscriberSystem<IGN_TYPE>::HandleMessage, this)) {
      ignerr << "Error subscribing to topic: " << topic_name_
             << "\n Ignition Subscriber will not work" << std::endl;
    }
  }

  ~IgnSubscriberSystem() override {}

  std::unique_ptr<DiscreteValues<double>> AllocateDiscreteState()
      const override {
    // Only make discrete states if we are outputting vector values.
    if (converter_->handles_discrete_values()) {
      std::vector<std::unique_ptr<BasicVector<double>>> discrete_state_vec(2);
      discrete_state_vec[kStateIndexMessage] =
          this->AllocateDiscreteOutputValue();
      discrete_state_vec[kStateIndexMessageCount] =
          std::make_unique<BasicVector<double>>(1);
      return std::make_unique<DiscreteValues<double>>(
          std::move(discrete_state_vec));
    }
    return LeafSystem<double>::AllocateDiscreteState();
  }

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    // Only make abstract states if we are outputting abstract message.
    if (!converter_->handles_discrete_values()) {
      std::vector<std::unique_ptr<AbstractValue>> abstract_vals(2);
      abstract_vals[kStateIndexMessage] = this->AllocateAbstractOutputValue();
      abstract_vals[kStateIndexMessageCount] = AbstractValue::Make<int>(0);
      return std::make_unique<AbstractValues>(std::move(abstract_vals));
    }
    return LeafSystem<double>::AllocateAbstractState();
  }

  void SetDefaultState(const Context<double>&,
                       State<double>* state) const override {
    DELPHYNE_DEMAND(state != nullptr);
    if (converter_->handles_discrete_values()) {
      ProcessMessageAndStoreToDiscreteState(
          &state->get_mutable_discrete_state());
    } else {
      ProcessMessageAndStoreToAbstractState(
          &state->get_mutable_abstract_state());
    }
  }

  /// Returns the topic name it publishes to.
  inline const std::string& get_topic_name() { return topic_name_; }

  /// Returns the message counter stored in @p context.
  int GetMessageCount(const Context<double>& context) const {
    return static_cast<int>(
        context.get_discrete_state(kStateIndexMessageCount).GetAtIndex(0));
  }

 protected:
  // Callback invoked each time a new message is received in the ignition
  // topic we are subscribed to. Set it as the last received message and
  // increase the received message count
  //
  // @param[in] ignition_message The message received.
  void HandleMessage(const IGN_TYPE& ignition_message) {
    std::lock_guard<std::mutex> lock(received_message_mutex_);
    last_received_message_ = ignition_message;
    received_message_count_++;
    received_message_condition_variable_.notify_all();
  }

  // Allocates the object that will be used as a value in the output port.
  // Returns nullptr if the converter handles abstract values.
  //
  // @return A pointer to a BasicVector object that will be used as the output
  // value or a nullptr if this converter doesn't handle discrete values.
  std::unique_ptr<BasicVector<double>> AllocateDiscreteOutputValue() const {
    return converter_->AllocateDiscreteOutputValue();
  }

  void DoCalcNextUpdateTime(
      const Context<double>& context,
      drake::systems::CompositeEventCollection<double>* events,
      double* time) const override {
    DELPHYNE_DEMAND(events != nullptr);
    DELPHYNE_DEMAND(time != nullptr);

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
      if (converter_->handles_discrete_values()) {
        drake::systems::EventCollection<
            drake::systems::DiscreteUpdateEvent<double>>& du_events =
            events->get_mutable_discrete_update_events();
        du_events.add_event(
            std::make_unique<drake::systems::DiscreteUpdateEvent<double>>(
                drake::systems::Event<double>::TriggerType::kTimed));
      } else {
        drake::systems::EventCollection<
            drake::systems::UnrestrictedUpdateEvent<double>>& uu_events =
            events->get_mutable_unrestricted_update_events();
        uu_events.add_event(
            std::make_unique<drake::systems::UnrestrictedUpdateEvent<double>>(
                drake::systems::Event<double>::TriggerType::kTimed));
      }
    }
  }

  void DoCalcUnrestrictedUpdate(
      const Context<double>&,
      const std::vector<
          const drake::systems::UnrestrictedUpdateEvent<double>*>&,
      State<double>* state) const override {
    DELPHYNE_DEMAND(state != nullptr);

    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<double>&,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
      DiscreteValues<double>* discrete_state) const override {
    DELPHYNE_DEMAND(discrete_state != nullptr);

    ProcessMessageAndStoreToDiscreteState(discrete_state);
  }

  void ProcessMessageAndStoreToDiscreteState(
      DiscreteValues<double>* discrete_state) const {
    DELPHYNE_DEMAND(discrete_state != nullptr);

    if (received_message_count_ > 0) {
      drake::systems::VectorBase<double>* vector_base =
          &discrete_state->get_mutable_vector(kStateIndexMessage);
      converter_->ProcessDiscreteOutput(last_received_message_, vector_base);
    }
    discrete_state->get_mutable_vector(kStateIndexMessageCount)
        .SetAtIndex(0, received_message_count_);
  }

  void ProcessMessageAndStoreToAbstractState(
      AbstractValues* abstract_state) const {
    DELPHYNE_DEMAND(abstract_state != nullptr);

    std::lock_guard<std::mutex> lock(received_message_mutex_);
    if (received_message_count_ > 0) {
      AbstractValue* abstract_value =
          &abstract_state->get_mutable_value(kStateIndexMessage);
      converter_->ProcessAbstractOutput(last_received_message_, abstract_value);
    }
    abstract_state->get_mutable_value(kStateIndexMessageCount)
        .GetMutableValue<int>() = received_message_count_;
  }

  void CalcDiscreteOutputValue(const Context<double>& context,
                               BasicVector<double>* output_vector) const {
    DELPHYNE_DEMAND(output_vector != nullptr);

    output_vector->SetFrom(context.get_discrete_state(kStateIndexMessage));
  }

  std::unique_ptr<AbstractValue> AllocateAbstractOutputValue() const {
    return converter_->AllocateAbstractDefaultValue();
  }

  void CalcAbstractOutputValue(const Context<double>& context,
                               AbstractValue* output_value) const {
    DELPHYNE_DEMAND(output_value != nullptr);

    output_value->SetFrom(
        context.get_abstract_state().get_value(kStateIndexMessage));
  }

 private:
  // The topic on which to publish ign-transport messages.
  const std::string topic_name_;

  // Ignition transport node.
  ignition::transport::Node node_;

  // The mutex that guards last_received_message_ and received_message_count_.
  mutable std::mutex received_message_mutex_;

  // A condition variable that's signaled every time the handler is called.
  mutable std::condition_variable received_message_condition_variable_;

  // The bytes of the most recently received LCM message.
  IGN_TYPE last_received_message_;

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  // Converts an ignition message to an output port value.
  mutable std::unique_ptr<IgnitionMessageConverter<IGN_TYPE>> converter_{};

  // The index of the state used to access the the message
  static constexpr int kStateIndexMessage = 0;

  // The index of the state used to access the the message count
  static constexpr int kStateIndexMessageCount = 1;
};

}  // namespace backend
}  // namespace delphyne
