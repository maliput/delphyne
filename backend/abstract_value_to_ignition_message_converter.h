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

#include "drake/systems/framework/context.h"

#include "backend/ign_publisher_system.h"
#include "backend/ign_to_lcm_translation.h"
#include "backend/ignition_message_converter.h"
#include "backend/lcm_to_ign_translation.h"
#include "backend/system.h"

using drake::systems::AbstractValue;

namespace delphyne {
namespace backend {

/// This class is a specialization of IgnitionMessageConverter that handles
/// abstract input ports. The input values are then casted to the
/// LCM_TYPE template parameter and further converted to an ignition message.
///
/// @tparam IGN_TYPE must be a valid ignition message type.
///
/// @tparam LCM_TYPE must be a valid LCM message type.
///
/// TODO(basicNew): We are abusing the language here a bit, as we may have an
/// abstract converter that has nothing to do with LCM-IGN translation. Keeping
/// it simple for the time being, but in the future we may have yet another
/// level in this class hierarchy.
template <class IGN_TYPE, class LCM_TYPE>
class AbstractValueToIgnitionMessageConverter
    : public IgnitionMessageConverter<IGN_TYPE> {
 public:
  /// @see IgnitionMessageConverter::handles_discrete_values
  ///
  /// This is an abstract converter, so it doesn't handle vector-based values.
  bool handles_discrete_values() override { return false; }

  /// @see IgnitionMessageConverter::AllocateAbstractDefaultValue
  ///
  /// @return An empty allocated value of the expected LCM type to translate.
  std::unique_ptr<AbstractValue> AllocateAbstractDefaultValue() const override {
    return std::make_unique<drake::systems::Value<LCM_TYPE>>(LCM_TYPE{});
  }

  void ProcessAbstractOutput(const IGN_TYPE& ign_message,
                             AbstractValue* output_value) override {
    DELPHYNE_DEMAND(output_value != nullptr);
    LCM_TYPE& message = output_value->GetMutableValue<LCM_TYPE>();
    IgnToLcm(ign_message, &message);
  }

  void ProcessInput(const IgnPublisherSystem<IGN_TYPE>* publisher,
                    const drake::systems::Context<double>& context,
                    int port_index, IGN_TYPE* ign_message) override {
    DELPHYNE_DEMAND(port_index >= 0);
    DELPHYNE_DEMAND(publisher != nullptr);
    DELPHYNE_DEMAND(ign_message != nullptr);

    // Just cast the current input to the expected LCM type and convert it
    // to an ignition message.
    // TODO(basicNew): get rid of the `lcmToIgn` call and do proper subclasses.
    const AbstractValue* input =
        publisher->EvalAbstractInput(context, port_index);
    const LCM_TYPE lcm_msg = input->GetValue<LCM_TYPE>();
    LcmToIgn(lcm_msg, ign_message);
  }

  int get_vector_size() override { return 0; }

 protected:
  // Do the actual conversion from the LCM message to the ignition message.
  //
  // @param[in] lcm_message The LCM message retrieved from the input port.
  //
  // @param[out] ign_message The ignition message, populated with the values
  // from the LCM messge.
  virtual void LcmToIgn(const LCM_TYPE& lcm_message, IGN_TYPE* ign_message) = 0;

  // Do the actual conversion from an ignition message to an LCM message.
  //
  // @param[in] ign_message The ignition message that we need to convert.
  //
  // @param[out] lcm_message The LCM message filled with the ign_message values.
  virtual void IgnToLcm(const IGN_TYPE& ign_message, LCM_TYPE* lcm_message) = 0;

  std::vector<float> ignToVector(const ignition::msgs::Vector3d& position) {
    return {static_cast<float>(position.x()), static_cast<float>(position.y()),
            static_cast<float>(position.z())};
  }

  std::vector<float> ignToVector(const ignition::msgs::Quaternion& orientation) {
    return {
        static_cast<float>(orientation.w()), static_cast<float>(orientation.x()),
        static_cast<float>(orientation.y()), static_cast<float>(orientation.z())};
  }

  int64_t millisFromSecs(int64_t secs) { return secs * 1000; }

  int64_t millisFromNsecs(int64_t nsecs) { return nsecs / 1000000; }

  int64_t secsFromMillis(int64_t millis) { return millis / 1000; }

  int64_t nsecsFromMillis(int64_t millis) { return millis % 1000 * 1000000; }

  int64_t secsFromMicros(int64_t micros) { return micros / 1000000; }

  int64_t nsecsFromMicros(int64_t micros) { return micros % 1000000 * 1000; }
};

}  // namespace backend
}  // namespace delphyne
