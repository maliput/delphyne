// Copyright 2018 Toyota Research Institute

#pragma once

#include <memory>

#include <drake/systems/framework/leaf_system.h>

#include "delphyne/protobuf/simple_car_state.pb.h"

namespace delphyne {

/// @brief A system that takes a SimpleCarState_V and splits it into separate
/// SimpleCarState messages.
class SimpleCarState_v_Splitter : public drake::systems::LeafSystem<double> {
 public:
  explicit SimpleCarState_v_Splitter(int agents_number);

 private:
  /// @brief Sets the output value with the SimpleCarState message that's at the
  /// given index of the SimpleCarState_v.
  /// @param[in] context The simulation context.
  /// @param[in] output A pointer to an abstracted SimpleCarState message.
  void DoSplit(const drake::systems::Context<double>& context,
               drake::systems::AbstractValue* output, int agent_index) const;

  /// @brief Allocates an abstract value object.
  std::unique_ptr<drake::systems::AbstractValue> DoAlloc();
};

}  // namespace delphyne
