// Copyright 2017 Toyota Research Institute

#include "backend/driving_command_to_ignition_message_converter.h"

#include "backend/time_conversion.h"

using drake::automotive::DrivingCommand;
using drake::automotive::DrivingCommandIndices;

namespace delphyne {
namespace backend {

int DrivingCommandToIgnitionMessageConverter::get_vector_size() {
  return DrivingCommandIndices::kNumCoordinates;
}

void DrivingCommandToIgnitionMessageConverter::VectorToIgn(
    const DrivingCommand<double>& input_vector, double time,
    ignition::msgs::AutomotiveDrivingCommand* ign_message) {
  DELPHYNE_DEMAND(ign_message != nullptr);

  ign_message->mutable_time()->CopyFrom(SecsToIgnitionTime(time));

  ign_message->set_theta(input_vector.steering_angle());
  ign_message->set_acceleration(input_vector.acceleration());
}

void DrivingCommandToIgnitionMessageConverter::IgnToVector(
    const ignition::msgs::AutomotiveDrivingCommand& ign_message,
    DrivingCommand<double>* output_vector) {
  DELPHYNE_DEMAND(output_vector != nullptr);

  output_vector->set_steering_angle(ign_message.theta());
  output_vector->set_acceleration(ign_message.acceleration());
}

}  // namespace backend
}  // namespace delphyne
