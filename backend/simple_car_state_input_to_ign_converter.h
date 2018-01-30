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

#include <memory>
#include <vector>

#include "backend/vector_input_to_ign_converter.h"

#include <drake/automotive/simple_car.h>

#include <protobuf/simple_car_state.pb.h>

namespace delphyne {
namespace backend {

/// This class is a specialization of VectorToIgnConverter that knows how
/// to populate a SimpleCarState ignition message from an input vector.
class SimpleCarStateInputToIgnConverter
    : public VectorToIgnConverter<ignition::msgs::SimpleCarState> {
 public:
  explicit SimpleCarStateInputToIgnConverter(int size)
      : VectorToIgnConverter(size) {}

 protected:
  void vectorToIgn(const VectorBase<double>& input_vector, double time,
                   ignition::msgs::SimpleCarState* ign_message) override {
    const auto* const vector =
        dynamic_cast<const drake::automotive::SimpleCarState<double>*>(
            &input_vector);
    const int64_t secs = time;
    const int64_t nsecs = (time - secs) * 1000000;
    ign_message->mutable_time()->set_sec(secs);
    ign_message->mutable_time()->set_nsec(nsecs);
    ign_message->set_x(vector->x());
    ign_message->set_y(vector->y());
    ign_message->set_heading(vector->heading());
    ign_message->set_velocity(vector->velocity());
  };
};

}  // namespace backend
}  // namespace delphyne
