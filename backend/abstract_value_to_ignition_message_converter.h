// Copyright 2017 Toyota Research Institute

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
/// @tparam LCM_TYPE must be a valid LCM message type.
///
/// @tparam IGN_TYPE must be a valid ignition message type.
///
/// TODO(basicNew): We are abusing the language here a bit, as we may have an
/// abstract converter that has nothing to do with LCM-IGN translation. Keeping
/// it simple for the time being, but in the future we may have yet another
/// level in this class hierarchy.
template <class LCM_TYPE, class IGN_TYPE>
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
    ignToLcm(ign_message, &message);
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
    lcmToIgn(lcm_msg, ign_message);
  }

  int get_vector_size() override { return 0; }
};

}  // namespace backend
}  // namespace delphyne
