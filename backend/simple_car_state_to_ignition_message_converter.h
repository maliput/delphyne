// Copyright 2017 Toyota Research Institute

#pragma once

#include <drake/automotive/simple_car.h>

#include <protobuf/simple_car_state.pb.h>

#include "backend/discrete_value_to_ignition_message_converter.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// This class is a specialization of DiscreteValueToIgnitionMessageConverter
/// that knows how to populate a SimpleCarState ignition message from an input
/// vector.
class DELPHYNE_BACKEND_VISIBLE SimpleCarStateToIgnitionMessageConverter
    : public DiscreteValueToIgnitionMessageConverter<
          ignition::msgs::SimpleCarState,
          drake::automotive::SimpleCarState<double>> {
 public:
  int get_vector_size();

 protected:
  void VectorToIgn(
      const drake::automotive::SimpleCarState<double>& input_vector,
      double time, ignition::msgs::SimpleCarState* ign_message) override;

  void IgnToVector(
      const ignition::msgs::SimpleCarState& ign_message,
      drake::automotive::SimpleCarState<double>* output_vector) override;
};

}  // namespace backend
}  // namespace delphyne
