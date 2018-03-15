// Copyright 2018 Toyota Research Institute

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

  //
  // @brief Translates an @p lcm_message into a @p ign_message. All derived
  //        translators must implement this method with the actual translation.
  //        @p ign_message is guaranteed to not be null, but it is NOT
  //        re-constructed on each call: the same object is copied and passed
  //        on each call. This function must perform any required cleanup from
  //        the previous call.
  // @see DeclareAbstractOutputPort
  //
  // @param[in] time_ms The curent time, in milliseconds.
  //
  virtual void DoLcmToIgnTranslation(const LCM_TYPE& lcm_message,
                                     IGN_TYPE* ign_message, int64_t) const = 0;

  // @brief Initializes the system's input and output ports.
  //        Since this function contains virtual calls, in cannot be called from
  //        LcmToIgnTranslatorSystem's constructor: all derived classes must
  //        call this function once before using the translator system to
  //        prevent runtime errors. The recommended way of doing this is to
  //        place this call in the derived translator's constructor.

  // Overload for LCM_TYPEs that inherit from drake::systems::VectorBase (which have
  // an associated vector size).
  template <class T = LCM_TYPE>
  typename std::enable_if<
      std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      void>::type
  InitPorts(int vector_size) {
    DeclareInputPort(drake::systems::kVectorValued, vector_size);

    // Ingition types always go in abstract output ports.
    DeclareAbstractOutputPort(&LcmToIgnTranslatorSystem::CalcIgnMessage);
  }

  // Overload for LCM_TYPEs that do not inherit from drake::systems::VectorBase.
  template <class T = LCM_TYPE>
  typename std::enable_if<
      !std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      void>::type
  InitPorts() {
    DeclareAbstractInputPort();

    // Ingition types always go in abstract output ports.
    DeclareAbstractOutputPort(&LcmToIgnTranslatorSystem::CalcIgnMessage);
  }

  // Translation helper functions and constants, to be used by derived
  // translators.

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
    DELPHYNE_DEMAND(ign_message != nullptr);

    // Retrieves the LCM message from the input port.
    const LCM_TYPE& lcm_message = ReadInputPort(context);

    // And then translates to ignition.
    auto time_ms = static_cast<int64_t>(context.get_time()) * 1000;
    DoLcmToIgnTranslation(lcm_message, ign_message, time_ms);
  }

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
