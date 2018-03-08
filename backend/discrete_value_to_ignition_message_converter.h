// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>

#include "backend/ign_publisher_system.h"
#include "backend/ignition_message_converter.h"
#include "backend/system.h"

using drake::systems::BasicVector;
using drake::systems::VectorBase;

namespace delphyne {
namespace backend {

/// This class is a specialization of IgnitionMessageConverter that handles
/// only VectorBase input ports. Concrete subclasses only need to define
/// the vectorToIgn method, which does the actual conversion from the
/// input vector to an ignition message.
///
/// @tparam IGN_TYPE must be a valid ignition message type.
template <class IGN_TYPE, class VECTOR_BASE_TYPE>
class DiscreteValueToIgnitionMessageConverter
    : public IgnitionMessageConverter<IGN_TYPE> {
 public:
  /// @see IgnitionMessageConverter::handles_discrete_values
  ///
  /// This is an discrete converter, so we do handle discrete values.
  bool handles_discrete_values() override { return true; }

  /// @see IgnitionMessageConverter::AllocateAbstractDefaultValue
  ///
  /// @return An empty vector-based object
  std::unique_ptr<BasicVector<double>> AllocateDiscreteOutputValue()
      const override {
    return std::make_unique<VECTOR_BASE_TYPE>();
  }

  void ProcessDiscreteOutput(const IGN_TYPE& ign_message,
                             VectorBase<double>* output_vector) override {
    auto* const driving_command_vector =
        dynamic_cast<VECTOR_BASE_TYPE*>(output_vector);

    DELPHYNE_DEMAND(driving_command_vector != nullptr);

    IgnToVector(ign_message, driving_command_vector);
  }

  void ProcessInput(const IgnPublisherSystem<IGN_TYPE>* publisher,
                    const drake::systems::Context<double>& context,
                    int port_index, IGN_TYPE* ign_message) override {
    DELPHYNE_DEMAND(port_index >= 0);
    DELPHYNE_DEMAND(publisher != nullptr);
    DELPHYNE_DEMAND(ign_message != nullptr);

    const VectorBase<double>* const input_vector =
        publisher->EvalVectorInput(context, port_index);
    const auto* const vector =
        dynamic_cast<const VECTOR_BASE_TYPE*>(input_vector);

    DELPHYNE_DEMAND(vector != nullptr);

    VectorToIgn(*vector, context.get_time(), ign_message);
  }

 protected:
  // Do the actual conversion from the input vector to the ignition message.
  //
  // @param[in] input_vector The vector retrieved from the input port.
  //
  // @param[in] time The current simulation time.
  //
  // @param[out] ign_message The ignition message, populated with the values
  // from the input vector.
  virtual void VectorToIgn(const VECTOR_BASE_TYPE& input_vector, double time,
                           IGN_TYPE* ign_message) = 0;

  // Do the actual conversion from an ignition message to a vector-based object
  // that will be used as an output value.
  //
  // @param[in] ign_message The ignition message that we need to convert.
  //
  // @param[out] output_vector The vector filled with the ign_message values.
  virtual void IgnToVector(const IGN_TYPE& ign_message,
                           VECTOR_BASE_TYPE* output_vector) = 0;
};

}  // namespace backend
}  // namespace delphyne
