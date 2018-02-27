// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "drake/automotive/gen/driving_command.h"
#include "ignition/msgs.hh"
#include "protobuf/automotive_driving_command.pb.h"

#include "backend/discrete_value_to_ignition_message_converter.h"

using drake::automotive::DrivingCommand;
using drake::automotive::DrivingCommandIndices;

namespace delphyne {
namespace backend {

/// This class is a specialization of DiscreteValueToIgnitionMessageConverter
/// that knows how to populate a SimpleCarState ignition message from an input
/// vector.
class DrivingCommandToIgnitionMessageConverter
    : public DiscreteValueToIgnitionMessageConverter<
          ignition::msgs::AutomotiveDrivingCommand, DrivingCommand<double>> {
 public:
  int get_vector_size() { return DrivingCommandIndices::kNumCoordinates; }

 protected:
  void VectorToIgn(
      const DrivingCommand<double>& input_vector, double time,
      ignition::msgs::AutomotiveDrivingCommand* ign_message) override {
    DELPHYNE_DEMAND(ign_message != nullptr);

    const int64_t secs = time;
    const int64_t nsecs = (time - secs) * 1000000000;
    ign_message->mutable_time()->set_sec(secs);
    ign_message->mutable_time()->set_nsec(nsecs);
    ign_message->set_theta(input_vector.steering_angle());
    ign_message->set_acceleration(input_vector.acceleration());
  };

  void IgnToVector(const ignition::msgs::AutomotiveDrivingCommand& ign_message,
                   DrivingCommand<double>* output_vector) override {
    DELPHYNE_DEMAND(output_vector != nullptr);

    output_vector->set_steering_angle(ign_message.theta());
    output_vector->set_acceleration(ign_message.acceleration());
  };
};

}  // namespace backend
}  // namespace delphyne
