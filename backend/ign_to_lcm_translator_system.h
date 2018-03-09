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

#include <type_traits>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_base.h"

#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates LCM objects on it's single input port (which
///        will be discrete or abstract based on the type of the LCM object) to
///        an IGN object on it's single abstract output port. This is a base
///        class that provides the input-output port boilerplate and helper
///        translator functions: derived classes need to implement the actual
///        translation.
///
/// @tparam IGN_TYPE must be a valid ignition message type.
/// @tparam LCM_TYPE must be a valid LCM message type.
template <class IGN_TYPE, class LCM_TYPE>
class IgnToLcmTranslatorSystem : public drake::systems::LeafSystem<double> {
 protected:
  // @brief Translates an @p ign_message into a @p lcm_message. All derived
  //        translators must implement this method with the actual translation.
  virtual void DoIgnToLcmTranslation(const IGN_TYPE& ign_message,
                                     LCM_TYPE* lcm_message) const = 0;

  // @brief Since this function contains virtual calls, in cannot be called from
  //        IgnToLcmTranslatorSystem's constructor: all derived classes must
  //        call this function once before using the translator system to
  //        prevent runtime errors. The recommended way of doing this is to
  //        place this call in the derived translator's constructor.
  void InitPorts() {
    // Input port (always abstract for ignition types)
    DeclareAbstractInputPort();

    // Output port (discrete or abstract, depending on the type of LCM_TYPE)
    InitOutputPort();
  }

  // Translation helper functions and constants, to be used by derived
  // translators.

  // @brief Converts an ignition time message to an timestamp in milliseconds
  //        (which is what LCM uses).
  int64_t ignTimeToTimestamp(const ::ignition::msgs::Time& ign_time) const {
    return ign_time.sec() * 1000 + ign_time.nsec() / 1000000;
  }

  // @brief Convert an ignition position message to a vector of floats (LCM's
  //        type for positions).
  std::vector<float> ignPositionToVector(
      const ignition::msgs::Vector3d& position) const {
    return {static_cast<float>(position.x()), static_cast<float>(position.y()),
            static_cast<float>(position.z())};
  }

  // @brief Convert an ignition quaterion message to a vector of floats (LCM's
  //        type for orientations).
  std::vector<float> ignQuaternionToVector(
      const ignition::msgs::Quaternion& orientation) const {
    return {static_cast<float>(orientation.w()),
            static_cast<float>(orientation.x()),
            static_cast<float>(orientation.y()),
            static_cast<float>(orientation.z())};
  }

 private:
  // Depending on the type of LCM_TYPE (whether or not it inherits from
  // drake::systems::VectorBase), we need to declare a vector output port, or an
  // abstract output port. The problem is those functions perform assertions on
  // the signature of the function they are called with, so we get compiler
  // errors if a call to the function is compiled (even if the function call
  // would be later optimized out of the code!).
  // The solution to this issue is to use std::enable_if, which relies in SFINAE
  // to prevent compilation of ill-formed functions.
  // When (if) we switch to C++17, all of this can be replaced with a simple
  // constexpr if.

  // (nventuro) This extra template assignment looks like a no-op, but it's
  // present on all of the std::enable_if I've seen, and I couldn't get it to
  // work without it. Sadly, I don't know enough about the type system and
  // templates to explain why it's needed.
  template <class T = LCM_TYPE>
  typename std::enable_if<
      std::is_base_of<drake::systems::VectorBase<double>, T>::value>::type
  InitOutputPort() {
    DeclareVectorOutputPort(&IgnToLcmTranslatorSystem::CalcLcmMessage);
  }

  template <class T = LCM_TYPE>
  typename std::enable_if<
      !std::is_base_of<drake::systems::VectorBase<double>, T>::value>::type
  InitOutputPort() {
    DeclareAbstractOutputPort(&IgnToLcmTranslatorSystem::CalcLcmMessage);
  }

  // The translator has a single input port, and a single output port.
  const int kPortIndex = 0;

  // @brief Calculates the state of the system's output port.
  //
  // @param[in] context The Drake system context.
  // @param[out] lcm_message The LCM message that will be stored in the output
  //                         port.
  void CalcLcmMessage(const drake::systems::Context<double>& context,
                      LCM_TYPE* lcm_message) const {
    // Retrieves the ignition message from the input port

    const drake::systems::AbstractValue* input =
        EvalAbstractInput(context, kPortIndex);
    DELPHYNE_DEMAND(input != nullptr);

    const IGN_TYPE& ign_message = input->GetValue<IGN_TYPE>();

    // And then translates to ignition
    DoIgnToLcmTranslation(ign_message, lcm_message);
  }
};

}  // namespace backend
}  // namespace delphyne
