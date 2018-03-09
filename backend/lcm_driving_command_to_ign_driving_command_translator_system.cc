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

#include <map>
#include <string>

#include "backend/lcm_driving_command_to_ign_driving_command_translator_system.h"

namespace delphyne {
namespace backend {

LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::
    LcmDrivingCommandToIgnDrivingCommandTranslatorSystem() {
  InitPorts();
}

void LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::
    DoLcmToIgnTranslation(
        const drake::automotive::DrivingCommand<double>& lcm_message,
        ignition::msgs::AutomotiveDrivingCommand* ign_message) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  auto time_ms = static_cast<int64_t>(translation_context->get_time()) * 1000;

  LcmTimestampToIgnition(time_ms, ign_message->mutable_time());
  ign_message->set_theta(lcm_message.steering_angle());
  ign_message->set_acceleration(lcm_message.acceleration());
}

int LcmDrivingCommandToIgnDrivingCommandTranslatorSystem::GetVectorSize()
    const {
  return drake::automotive::DrivingCommandIndices::kNumCoordinates;
}

}  // namespace backend
}  // namespace delphyne
