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
/// @tparam LCM_TYPE must be a valid LCM message type.
/// @tparam IGN_TYPE must be a valid ignition message type.
template <class LCM_TYPE, class IGN_TYPE>
class LcmToIgnTranslatorSystem : public drake::systems::LeafSystem<double> {
 protected:
  // @brief Derived translators for discrete LCM types (types that inherit from
  //        VectorBase) must override this method.
  virtual int GetVectorSize() const { DELPHYNE_ABORT(); }

  // @brief Translates an @p lcm_message into a @p ign_message. All derived
  //        translators must implement this method with the actual translation.
  virtual void DoLcmToIgnTranslation(const LCM_TYPE& lcm_message,
                                     IGN_TYPE* ign_message) const = 0;

  // @brief Since this function contains virtual calls, in cannot be called from
  //        LcmToIgnTranslatorSystem's constructor: all derived classes must
  //        call this function once before using the translator system to
  //        prevent runtime errors. The recommended way of doing this is to
  //        place this call in the derived translator's constructor.
  void InitPorts() {
    // Input port (discrete or abstract, depending on the type of LCM_TYPE)
    if (IsVectorBasedTranslator()) {
      DeclareInputPort(drake::systems::kVectorValued, GetVectorSize());
    } else {
      DeclareAbstractInputPort();
    }

    // Output port (always abstract for ignition types)
    DeclareAbstractOutputPort(&LcmToIgnTranslatorSystem::CalcIgnMessage);
  }

  // Translation helper functions and constants, to be used by derived
  // translators.

  mutable drake::systems::Context<double> const* translation_context;

  const unsigned int kPositionVectorSize{3};
  const unsigned int kOrientationVectorSize{4};

  // @brief Converts an array of floats (LCM's type for positions) to an
  //        ignition position message.
  //
  // @param[in] lcm_position The LCM position array.
  // @param[out] ign_position The ignition position message.
  void LcmPositionToIgnition(const float lcm_position[3],
                             ignition::msgs::Vector3d* ign_position) const {
    DELPHYNE_DEMAND(ign_position != nullptr);

    ign_position->set_x(lcm_position[0]);
    ign_position->set_y(lcm_position[1]);
    ign_position->set_z(lcm_position[2]);
  }

  // @brief Converts an array of floats (LCM's type for quaternions) to an
  //        ignition quaterion message.
  //
  // @param[in] lcm_quaternion The LCM orientation array.
  // @param[out] ign_quaternion The ign quaternion message.
  void LcmQuaternionToIgnition(
      const float lcm_quaternion[4],
      ignition::msgs::Quaternion* ign_quaternion) const {
    DELPHYNE_DEMAND(ign_quaternion != nullptr);

    ign_quaternion->set_w(lcm_quaternion[0]);
    ign_quaternion->set_x(lcm_quaternion[1]);
    ign_quaternion->set_y(lcm_quaternion[2]);
    ign_quaternion->set_z(lcm_quaternion[3]);
  }

  // @brief Converts an LCM timestamp (in milliseconds) to an ingition time
  //        message.
  //
  // @param[in] lcm_timestamp_ms The LCM timestamp (in milliseconds).
  // @param[out] ign_time The ignition time message.
  void LcmTimestampToIgnition(int64_t lcm_timestamp_ms,
                              ::ignition::msgs::Time* ign_time) const {
    DELPHYNE_DEMAND(ign_time != nullptr);

    ign_time->set_sec(lcm_timestamp_ms / 1000);
    ign_time->set_nsec(lcm_timestamp_ms % 1000 * 1000000);
  }

 private:
  constexpr bool IsVectorBasedTranslator() const {
    return std::is_base_of<drake::systems::VectorBase<double>, LCM_TYPE>::value;
  }

  // The translator has a single input port, and a single output port.
  const int kPortIndex = 0;

  // @brief Calculates the state of the system's output port.
  //
  // @param[in] context The Drake system context.
  // @param[out] ign_message The ignition message that will be stored in the
  //                         output port.
  void CalcIgnMessage(const drake::systems::Context<double>& context,
                      IGN_TYPE* ign_message) const {
    translation_context = &context;

    // Retrieves the LCM message from the input port
    const LCM_TYPE& lcm_message = ReadInputPort(context);

    // And then translates to ignition
    DoLcmToIgnTranslation(lcm_message, ign_message);
  }

  // Depending on the type of LCM_TYPE (whether or not it inherits from
  // drake::systems::VectorBase), we need to read from a vector input port, or
  // from an abstract output port. The problem is those functions perform
  // assertions on the inferred return type, so we get compiler errors if a call
  // to the function is compiled (even if the function call would be later
  // optimized out of the code!).
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
      std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      const LCM_TYPE&>::type
  ReadInputPort(const drake::systems::Context<double>& context) const {
    const drake::systems::VectorBase<double>* const input_vector =
        EvalVectorInput(context, kPortIndex);
    DELPHYNE_DEMAND(input_vector != nullptr);

    const LCM_TYPE* const vector = dynamic_cast<const LCM_TYPE*>(input_vector);
    DELPHYNE_DEMAND(vector != nullptr);

    return *vector;
  }

  template <class T = LCM_TYPE>
  typename std::enable_if<
      !std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      const LCM_TYPE&>::type
  ReadInputPort(const drake::systems::Context<double>& context) const {
    const drake::systems::AbstractValue* input =
        EvalAbstractInput(context, kPortIndex);
    DELPHYNE_DEMAND(input != nullptr);

    return input->GetValue<LCM_TYPE>();
  }
};

}  // namespace backend
}  // namespace delphyne
