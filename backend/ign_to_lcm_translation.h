// Copyright 2017 Open Source Robotics Foundation
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

namespace ignition {
namespace msgs {
class AutomotiveDrivingCommand;
class Model_V;
}
}

namespace drake {
class lcmt_driving_command_t;
class lcmt_viewer_draw;
}

namespace delphyne {
namespace backend {

/// \brief Translate a driving command from LCM to ignition
/// \param[in]  ign_driving_command An ignition message containing the driving
/// command
/// \param[out] lcm_driving_command The resulting LCM command
void ignToLcm(
    const ignition::msgs::AutomotiveDrivingCommand& ign_driving_command,
    drake::lcmt_driving_command_t* lcm_driving_command);

/// \brief Translate a model vector ignition message to an LCM view draw message
/// \param[in]  robot_models An ignition message containing the model vector
/// \param[out] robot_draw_data The resulting LCM view draw message
void ignToLcm(const ignition::msgs::Model_V& robot_models,
              drake::lcmt_viewer_draw* robot_draw_data);

}  // namespace backend
}  // namespace delphyne
