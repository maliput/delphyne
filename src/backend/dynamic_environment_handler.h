// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#pragma once

#include <maliput/api/road_network.h>
#include <maliput/common/maliput_copyable.h>

#include "delphyne/macros.h"

namespace delphyne {

/// Abstract API for managing the rules dynamic states of a maliput::api::RoadNetwork.
/// The states are expected to change based on time.
class DynamicEnvironmentHandler {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(DynamicEnvironmentHandler)
  DynamicEnvironmentHandler() = delete;

  virtual ~DynamicEnvironmentHandler() = default;

  /// Updates the rule's states.
  /// @param sim_time Current simulation time.
  virtual void Update(double sim_time) = 0;

 protected:
  /// Creates DynamicEnvironmentHandler
  /// @param road_network maliput::api::RoadNetwork pointer.
  DynamicEnvironmentHandler(maliput::api::RoadNetwork* road_network) : road_network_(road_network) {
    DELPHYNE_VALIDATE(road_network_ != nullptr, std::invalid_argument, "RoadNetwork can't be nullptr.");
  }

  maliput::api::RoadNetwork* road_network_{nullptr};
};

}  // namespace delphyne
