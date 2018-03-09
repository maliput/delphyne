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

#include "backend/lcm_to_ign_translator_system.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief A system that translates LCM driving command messages to ignition
/// driving command messages.
class DELPHYNE_BACKEND_VISIBLE
    LcmDrivingCommandToIgnDrivingCommandTranslatorSystem
    : public LcmToIgnTranslatorSystem<
          drake::automotive::DrivingCommand<double>,
          ignition::msgs::AutomotiveDrivingCommand> {
 public:
  /// @brief Default constructor. @see LcmToIgnTranslatorSystem::InitPorts.
  LcmDrivingCommandToIgnDrivingCommandTranslatorSystem();

 protected:
  // @brief @see LcmToIgnTranslatorSystem::DoLcmToIgnTranslation.
  void DoLcmToIgnTranslation(
      const drake::automotive::DrivingCommand<double>& lcm_message,
      ignition::msgs::AutomotiveDrivingCommand* ign_message) const override;

  int GetVectorSize() const override;
};

}  // namespace backend
}  // namespace delphyne
