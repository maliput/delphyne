// Copyright 2018 Toyota Research Institute

#pragma once

#include <type_traits>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_base.h"

#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates Drake messages on its single input port
///        (which will be discrete or abstract based on the type of the Drake
///        message) to an ignition message on its single abstract output port.
///        This is a base class that provides the input-output port boilerplate
///        and helper translator functions: derived classes need to implement
///        the actual translation.
///
/// @tparam DRAKE_TYPE must be a valid Drake message type.
/// @tparam IGN_TYPE must be a valid ignition message type.
template <class DRAKE_TYPE, class IGN_TYPE>
class DrakeToIgnTranslatorSystem : public drake::systems::LeafSystem<double> {
 public:
  // Two constructors exist, but only one is enabled, depending on DRAKE_TYPE.

  // Constructor for translators with a DRAKE_TYPE that inherits from
  // drake::systems::VectorBase.
  template <class T = DRAKE_TYPE>
  DrakeToIgnTranslatorSystem(
      int vector_size,
      typename std::enable_if<
          std::is_base_of<drake::systems::VectorBase<double>, T>::value,
          void>::type* = 0) {
    // Vector input port.
    DeclareInputPort(drake::systems::kVectorValued, vector_size);

    // Output port (abstract for all ignition types).
    DeclareAbstractOutputPort(&DrakeToIgnTranslatorSystem::CalcIgnMessage);
  }

  // Constructor for translators with a DRAKE_TYPE that does not inherit from
  // drake::systems::VectorBase.
  template <class T = DRAKE_TYPE>
  DrakeToIgnTranslatorSystem(
      typename std::enable_if<
          !std::is_base_of<drake::systems::VectorBase<double>, T>::value,
          void>::type* = 0) {
    // Abstract input port.
    DeclareAbstractInputPort();

    // Output port (abstract for all ignition types).
    DeclareAbstractOutputPort(&DrakeToIgnTranslatorSystem::CalcIgnMessage);
  }

 protected:
  // @brief Translates a @p drake_message into an @p ign_message. All derived
  //        translators must implement this method with the actual translation.
  //        @p ign_message is guaranteed to not be null, but it is NOT
  //        re-constructed on each call: the same object is copied and passed
  //        on each call. This function must perform any required cleanup from
  //        the previous call.
  // @see DeclareAbstractOutputPort
  //
  // @param[in] time_ms The curent time, in milliseconds.
  virtual void DoDrakeToIgnTranslation(const DRAKE_TYPE& drake_message,
                                     IGN_TYPE* ign_message, int64_t) const = 0;

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

  // @brief Converts an LCM timestamp (in milliseconds) to an ignition time
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

    // Retrieves the Drake message from the input port.
    const DRAKE_TYPE& drake_message = ReadInputPort(context);

    // And then translates to ignition.
    auto time_ms = static_cast<int64_t>(context.get_time()) * 1000;
    DoDrakeToIgnTranslation(drake_message, ign_message, time_ms);
  }

  // Depending on the type of DRAKE_TYPE (whether or not it inherits from
  // drake::systems::VectorBase), we need to read from a vector input port, or
  // from an abstract output port. The problem is those functions perform
  // assertions on the inferred return type, so we get compiler errors if a call
  // to the wrong function is compiled (e.g. a call to read a vector input port
  // into a non-vector object).
  // The solution to this issue is to use std::enable_if, which relies in SFINAE
  // to prevent compilation of ill-formed overloads.
  // When (if) we switch to C++17, all of this can be replaced with a simple
  // constexpr if.


  // @brief Reads an input port for Drake objects that inherit from VectorBase.
  template <class T = DRAKE_TYPE>
  typename std::enable_if<
      std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      const DRAKE_TYPE&>::type
  ReadInputPort(const drake::systems::Context<double>& context) const {
    const drake::systems::VectorBase<double>* const input_vector =
        EvalVectorInput(context, kPortIndex);
    DELPHYNE_DEMAND(input_vector != nullptr);

    const DRAKE_TYPE* const vector =
        dynamic_cast<const DRAKE_TYPE*>(input_vector);
    DELPHYNE_DEMAND(vector != nullptr);

    return *vector;
  }

  // @brief Reads an input port for Drake objects that do not inherit from
  //        VectorBase.
  template <class T = DRAKE_TYPE>
  typename std::enable_if<
      !std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      const DRAKE_TYPE&>::type
  ReadInputPort(const drake::systems::Context<double>& context) const {
    const drake::systems::AbstractValue* input =
        EvalAbstractInput(context, kPortIndex);
    DELPHYNE_DEMAND(input != nullptr);

    return input->GetValue<DRAKE_TYPE>();
  }
};

}  // namespace backend
}  // namespace delphyne
