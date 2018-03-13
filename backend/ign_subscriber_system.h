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

#include "backend/system.h"

using drake::systems::Value;
using drake::systems::AbstractValue;
using drake::systems::AbstractValues;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::State;

namespace delphyne {
namespace backend {

/// This class is the counterpart of Drake's LcmSubscriberSystem. Most of the
/// code has been taken from that class and adapted to the types of values
/// that we use.
///
/// @tparam IGN_TYPE must be a valid ignition message type.
template <class IGN_TYPE>
class IgnSubscriberSystem : public LeafSystem<double> {
 public:
  /// Default constructor.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  /// be subscribed to.
  explicit IgnSubscriberSystem(const std::string& topic_name)
      : topic_name_(topic_name) {
    DeclareAbstractOutputPort(
        [this](const Context<double>&) {
          return this->AllocateDefaultAbstractValue();
        },
        [this](const Context<double>& context, AbstractValue* out) {
          this->IgnSubscriberSystem::CalcIgnMessage(context, out);
        });

    if (!node_.Subscribe(topic_name_,
                         &IgnSubscriberSystem<IGN_TYPE>::HandleMessage, this)) {
      ignerr << "Error subscribing to topic: " << topic_name_
             << "\n Ignition Subscriber will not work" << std::endl;
    }
  }

  ~IgnSubscriberSystem() override {}

  std::unique_ptr<AbstractValue> AllocateDefaultAbstractValue() const {
    return std::make_unique<drake::systems::Value<IGN_TYPE>>(IGN_TYPE{});
  }

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    std::vector<std::unique_ptr<AbstractValue>> abstract_vals(2);
    abstract_vals[kStateIndexMessage] = AllocateDefaultAbstractValue();
    abstract_vals[kStateIndexMessageCount] = AbstractValue::Make<int>(0);
    return std::make_unique<AbstractValues>(std::move(abstract_vals));
  }

  void SetDefaultState(const Context<double>&,
                       State<double>* state) const override {
    DELPHYNE_DEMAND(state != nullptr);
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

  /// Returns the topic name it is subscribed to.
  inline const std::string& get_topic_name() { return topic_name_; }

  /// Returns the message counter stored in @p context.
  int GetMessageCount(const Context<double>& context) const {
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

      drake::systems::EventCollection<
          drake::systems::UnrestrictedUpdateEvent<double>>& uu_events =
          events->get_mutable_unrestricted_update_events();

      uu_events.add_event(
          std::make_unique<drake::systems::UnrestrictedUpdateEvent<double>>(
              drake::systems::Event<double>::TriggerType::kTimed));
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

  void ProcessMessageAndStoreToAbstractState(
      AbstractValues* abstract_state) const {
    DELPHYNE_DEMAND(abstract_state != nullptr);

    std::lock_guard<std::mutex> lock(received_message_mutex_);

    if (received_message_count_ > 0) {
      abstract_state->get_mutable_value(kStateIndexMessage)
          .GetMutableValue<IGN_TYPE>() = last_received_message_;
    }

    abstract_state->get_mutable_value(kStateIndexMessageCount)
        .GetMutableValue<int>() = received_message_count_;
  }

  void CalcIgnMessage(const Context<double>& context,
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

  // The bytes of the most recently received LCM message.
  IGN_TYPE last_received_message_;

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  // The index of the state used to access the the message
  static constexpr int kStateIndexMessage = 0;

  // The index of the state used to access the the message count
  static constexpr int kStateIndexMessageCount = 1;
};

}  // namespace backend
}  // namespace delphyne
