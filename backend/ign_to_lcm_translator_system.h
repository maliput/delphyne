// Copyright 2018 Toyota Research Institute

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
  //        @p lcm_message is NOT re-constructed on each call: the same object
  //        is copied and passed again. This function must perform any required
  //        cleanup from the previous call. @see DeclareAbstractOutputPort
  //        @see DeclareVectorOutputPort
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
  // drake::systems::VectorBase), we need to read from a vector input port, or
  // from an abstract output port. The problem is those functions perform
  // assertions on the inferred return type, so we get compiler errors if a call
  // to the function is compiled.
  // The solution to this issue is to use std::enable_if, which relies in SFINAE
  // to prevent compilation of ill-formed overloads.
  // When (if) we switch to C++17, all of this can be replaced with a simple
  // constexpr if.
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
