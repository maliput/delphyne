// Copyright 2017 Toyota Research Institute

#include "backend/simple_car_state_to_ignition_message_converter.h"

#include <drake/automotive/simple_car.h>

#include <protobuf/simple_car_state.pb.h>

#include "backend/discrete_value_to_ignition_message_converter.h"
#include "backend/time_conversion.h"

using drake::automotive::SimpleCarState;
using drake::automotive::SimpleCarStateIndices;

namespace delphyne {
namespace backend {

int SimpleCarStateToIgnitionMessageConverter::get_vector_size() {
  return SimpleCarStateIndices::kNumCoordinates;
}

void SimpleCarStateToIgnitionMessageConverter::VectorToIgn(
    const SimpleCarState<double>& input_vector, double time,
    ignition::msgs::SimpleCarState* ign_message) {
  DELPHYNE_DEMAND(ign_message != nullptr);

  ign_message->mutable_time()->CopyFrom(SecsToIgnitionTime(time));

  ign_message->set_x(input_vector.x());
  ign_message->set_y(input_vector.y());
  ign_message->set_heading(input_vector.heading());
  ign_message->set_velocity(input_vector.velocity());
}

void SimpleCarStateToIgnitionMessageConverter::IgnToVector(
    const ignition::msgs::SimpleCarState& ign_message,
    SimpleCarState<double>* output_vector) {
  DELPHYNE_DEMAND(output_vector != nullptr);

  output_vector->set_x(ign_message.x());
  output_vector->set_y(ign_message.y());
  output_vector->set_heading(ign_message.heading());
  output_vector->set_velocity(ign_message.velocity());
}

}  // namespace backend
}  // namespace delphyne
