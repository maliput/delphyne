// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "backend/load_robot_aggregator.h"

#include <vector>

#include <maliput/common/maliput_unused.h>

#include "delphyne/macros.h"

namespace delphyne {

LoadRobotAggregator::LoadRobotAggregator(const std::vector<drake::lcmt_viewer_load_robot>& load_robot_messages)
    : load_robot_messages_(load_robot_messages) {
  DeclareAbstractOutputPort(&LoadRobotAggregator::CalcAggregatedLoadRobot);
}

void LoadRobotAggregator::CalcAggregatedLoadRobot(const drake::systems::Context<double>& context,
                                                  drake::lcmt_viewer_load_robot* load_robot_message) const {
  maliput::common::unused(context);
  DELPHYNE_VALIDATE(load_robot_message != nullptr, std::invalid_argument,
                    "Load robot message pointer must not be null");

  // Clears state from the previous call.
  // @see DeclareAbstractOutputPort
  load_robot_message->link.clear();

  for (const auto& message : load_robot_messages_) {
    for (const drake::lcmt_viewer_link_data& link : message.link) {
      load_robot_message->link.push_back(link);
    }
  }

  load_robot_message->num_links = load_robot_message->link.size();
}

}  // namespace delphyne
