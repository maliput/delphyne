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
#include <vector>

#include "drake/systems/framework/context.h"

#include "backend/ign_publisher_system.h"
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

  // Helper translation functions to be used by derived classes

  std::vector<float> ignToVector(const ignition::msgs::Vector3d& position) {
    return {static_cast<float>(position.x()), static_cast<float>(position.y()),
            static_cast<float>(position.z())};
  }

  std::vector<float> ignToVector(
      const ignition::msgs::Quaternion& orientation) {
    return {static_cast<float>(orientation.w()),
            static_cast<float>(orientation.x()),
            static_cast<float>(orientation.y()),
            static_cast<float>(orientation.z())};
  }

  void lcmToIgn(const float positionData[3],
                ignition::msgs::Vector3d* positionModel) {
    positionModel->set_x(positionData[0]);
    positionModel->set_y(positionData[1]);
    positionModel->set_z(positionData[2]);
  }

  void lcmToIgn(const float quaternionData[4],
                ignition::msgs::Quaternion* quaternionModel) {
    quaternionModel->set_w(quaternionData[0]);
    quaternionModel->set_x(quaternionData[1]);
    quaternionModel->set_y(quaternionData[2]);
    quaternionModel->set_z(quaternionData[3]);
  }

  void ignToLcm(const ::ignition::msgs::Time& ign_time,
                int64_t* lcm_timestamp_ms) {
    *lcm_timestamp_ms = ign_time.sec() * 1000 + ign_time.nsec() / 1000000;
  }

  void lcmToIgn(int64_t lcm_timestamp_ms, ::ignition::msgs::Time* ign_time) {
    ign_time->set_sec(lcm_timestamp_ms / 1000);
    ign_time->set_nsec(lcm_timestamp_ms % 1000 * 1000000);
  }
};

}  // namespace backend
}  // namespace delphyne
