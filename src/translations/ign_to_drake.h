// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include <type_traits>
#include <vector>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/vector_base.h>

#include "delphyne/macros.h"

namespace delphyne {

/// @brief A system that translates ignition messages on its single abstract
/// input port to a Drake message on its single output port (which will be
/// vector based or abstract depending on the type of the Drake message). This
/// is a base class that provides the input-output port boilerplate and helper
/// translator functions: derived classes need to implement the actual
/// translation.
///
/// @tparam IGN_TYPE must be a valid ignition message type.
/// @tparam DRAKE_TYPE must be a valid Drake message type.
template <class IGN_TYPE, class DRAKE_TYPE>
class IgnToDrake : public drake::systems::LeafSystem<double> {
 public:
  IgnToDrake() {
    // Input port (abstract for all ignition types).
    DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<IGN_TYPE>());

    // Output port (vector or abstract, depending on DRAKE_TYPE).
    InitOutputPort();
  }

 protected:
  // @brief Translates an @p ign_message into a @p drake_message. All derived
  // translators must implement this method with the actual translation.
  // @p drake_message is guaranteed to not be null, but it is NOT ever
  // re-constructed: the same object is copied and passed on each call. This
  // function must perform any required cleanup from the previous call.
  // @see DeclareVectorOutputPort
  // @see DeclareAbstractOutputPort
  virtual void DoIgnToDrakeTranslation(const IGN_TYPE& ign_message, DRAKE_TYPE* drake_message) const = 0;

  // Translation helper functions and constants, to be used by derived
  // translators.

  // @brief Convert an ignition position message to a vector of floats (LCM's
  // type for positions).
  static std::vector<float> IgnPositionToVector(const ignition::msgs::Vector3d& position) {
    return {static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z())};
  }

  // @brief Convert an ignition quaterion message to a vector of floats (LCM's
  // type for orientations).
  static std::vector<float> IgnQuaternionToVector(const ignition::msgs::Quaternion& orientation) {
    return {static_cast<float>(orientation.w()), static_cast<float>(orientation.x()),
            static_cast<float>(orientation.y()), static_cast<float>(orientation.z())};
  }

 private:
  // Depending on DRAKE_TYPE (whether or not it inherits from
  // drake::systems::VectorBase), we need to write to a vector output port, or
  // to an abstract output port. The problem is Drake's functions perform
  // assertions on the written object type, so we get compiler errors if a call
  // to the wrong function is compiled (e.g. a call to write a vector output
  // port with a non-vector object).
  // The solution to this issue is to use std::enable_if, which relies in SFINAE
  // to prevent compilation of ill-formed overloads.
  // When (if) we switch to C++17, all of this can be replaced with a simple
  // constexpr if.

  // @brief Output port initialization for Drake objects that inherit from
  // VectorBase.
  template <class T = DRAKE_TYPE>
  typename std::enable_if<std::is_base_of<drake::systems::VectorBase<double>, T>::value>::type InitOutputPort() {
    DeclareVectorOutputPort(&IgnToDrake::CalcDrakeMessage);
  }

  // @brief Output port initialization for Drake objects that do not inherit
  // from VectorBase.
  template <class T = DRAKE_TYPE>
  typename std::enable_if<!std::is_base_of<drake::systems::VectorBase<double>, T>::value>::type InitOutputPort() {
    DeclareAbstractOutputPort(&IgnToDrake::CalcDrakeMessage);
  }

  // The translator has a single input port, and a single output port.
  const int kPortIndex = 0;

  // @brief Calculates the state of the system's output port.
  //
  // @param[in] context The Drake system context.
  // @param[out] drake_message The Drake message that will be stored in the
  // output port.
  void CalcDrakeMessage(const drake::systems::Context<double>& context, DRAKE_TYPE* drake_message) const {
    DELPHYNE_VALIDATE(drake_message != nullptr, std::invalid_argument, "Drake message pointer must not be null");

    // Retrieves the ignition message from the input port.
    const IGN_TYPE* ign_message = this->template EvalInputValue<IGN_TYPE>(context, kPortIndex);

    // And then translates to Drake.
    DoIgnToDrakeTranslation(*ign_message, drake_message);
  }
};

}  // namespace delphyne
