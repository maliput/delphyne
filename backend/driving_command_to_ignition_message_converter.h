// Copyright 2017 Toyota Research Institute

#pragma once

#include "drake/automotive/gen/driving_command.h"

#include "protobuf/automotive_driving_command.pb.h"

#include "backend/discrete_value_to_ignition_message_converter.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// This class is a specialization of DiscreteValueToIgnitionMessageConverter
/// that knows how to populate a SimpleCarState ignition message from an input
/// vector.
class DELPHYNE_BACKEND_VISIBLE DrivingCommandToIgnitionMessageConverter
    : public DiscreteValueToIgnitionMessageConverter<
          ignition::msgs::AutomotiveDrivingCommand,
          drake::automotive::DrivingCommand<double>> {
 public:
  int get_vector_size();

 protected:
  void VectorToIgn(
      const drake::automotive::DrivingCommand<double>& input_vector,
      double time,
      ignition::msgs::AutomotiveDrivingCommand* ign_message) override;

  void IgnToVector(
      const ignition::msgs::AutomotiveDrivingCommand& ign_message,
      drake::automotive::DrivingCommand<double>* output_vector) override;
};

}  // namespace backend
}  // namespace delphyne
