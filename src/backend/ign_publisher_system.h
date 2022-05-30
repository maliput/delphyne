// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
#include <maliput/common/maliput_unused.h>

#include "delphyne/macros.h"

namespace delphyne {

/// A system to publish ignition messages at its single abstract input
/// port through an ignition transport topic.
///
/// @tparam IGN_TYPE A valid ignition message type.
template <typename IGN_TYPE,
          typename std::enable_if<std::is_base_of<ignition::transport::ProtoMsg, IGN_TYPE>::value, int>::type = 0>
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
  explicit IgnPublisherSystem(const std::string& topic_name, double publish_rate) : topic_name_(topic_name) {
    DELPHYNE_VALIDATE(publish_rate > 0.0, std::invalid_argument, "Invalid publish rate (must be > 0.0)");
    this->DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<IGN_TYPE>());
    const double kPublishTimeOffset{0.};
    const drake::systems::PublishEvent<double> publish_event(
        drake::systems::Event<double>::TriggerType::kPeriodic,
        std::bind(&IgnPublisherSystem<IGN_TYPE>::PublishIgnMessage, this, std::placeholders::_1,
                  std::placeholders::_2));
    this->DeclarePeriodicEvent(1.0 / publish_rate, kPublishTimeOffset, publish_event);
    publisher_ = node_.Advertise<IGN_TYPE>(topic_name);
  }

  /// Constructs a publisher that forwards messages at the fastest possible
  /// rate (i.e. on every simulation step) to the given ignition transport
  /// topic.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  ///                       be publishing to.
  explicit IgnPublisherSystem(const std::string& topic_name) : topic_name_(topic_name) {
    this->DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<IGN_TYPE>());
    const drake::systems::PublishEvent<double> publish_event(
        drake::systems::Event<double>::TriggerType::kPerStep,
        std::bind(&IgnPublisherSystem<IGN_TYPE>::PublishIgnMessage, this, std::placeholders::_1,
                  std::placeholders::_2));
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
    maliput::common::unused(event);
    // Retrieves the input value from the sole input port.
    const int kPortIndex = 0;
    // Publishes the message onto the specified
    // ignition transport topic.
    publisher_.Publish(*this->template EvalInputValue<IGN_TYPE>(context, kPortIndex));
  }

  // The topic on which to publish ignition transport messages.
  const std::string topic_name_;

  // Ignition transport node.
  ignition::transport::Node node_;

  // Ignition transport publisher.
  ignition::transport::Node::Publisher publisher_;
};

}  // namespace delphyne
