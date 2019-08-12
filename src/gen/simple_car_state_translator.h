// Copyright 2018 Toyota Research Institute

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <memory>
#include <vector>

#include <drake/lcmt_simple_car_state_t.hpp>
#include <drake/systems/lcm/lcm_and_vector_base_translator.h>

#include "gen/simple_car_state.h"

namespace delphyne {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * SimpleCarState type.
 */
class SimpleCarStateTranslator final : public drake::systems::lcm::LcmAndVectorBaseTranslator {
 public:
  SimpleCarStateTranslator() : LcmAndVectorBaseTranslator(SimpleCarStateIndices::kNumCoordinates) {}
  std::unique_ptr<drake::systems::BasicVector<double>> AllocateOutputVector() const final;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   drake::systems::VectorBase<double>* vector_base) const final;
  void Serialize(double time, const drake::systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const final;
};

}  // namespace delphyne
