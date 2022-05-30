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

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>
#include <maliput/api/road_network.h>

#include "delphyne/macros.h"

namespace delphyne {

/// RightOfWaySystem evaluates the current RightOfWayRule state that applies to the agent's location.
/// It acts interfering the target velocity that is expected to be injected to the speed controller.
///  - The velocity output is zero when the rule indicates to stop.
///  - The velocity output pass through the input velocity  when the rule indicates to go.
///
/// Assumptions:
///   1) Only "Stop" and "Go" states are targeted.
///
/// Input Port 0: a LaneDirection representing the requested lane and direction
///   of travel.
///   (InputPort getter: lane_state_input())
///
/// Input Port 1: A PoseVector for the ego car.
///   (InputPort getter: pose_input())
///
/// Input Port 2: The target velocity to be injected to the plant.
///   (InputPort getter: velocity_input())
///
/// Output Port 0: Filtered velocity that could be either zero or velocity_input
///   depending on the state of the right-of-way rule.
///   (OutputPort getter: velocity_output())
///
/// @ingroup automotive_controllers
template <typename T>
class RightOfWaySystem final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWaySystem)

  /// Constructs a RightOfWaySystem
  /// @param road_network Pointer to a maliput::api::RoadNetwork.
  ///
  /// @throws std::invalid_argument When road_network is nullptr.
  explicit RightOfWaySystem(maliput::api::RoadNetwork* road_network);

  RightOfWaySystem() = delete;

  ~RightOfWaySystem() override = default;

  const drake::systems::InputPort<T>& lane_state_input() const;

  const drake::systems::InputPort<T>& velocity_input() const;

  const drake::systems::InputPort<T>& pose_input() const;

  const drake::systems::OutputPort<T>& velocity_output() const;

 protected:
  void CalcOutputVelocity(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;

 private:
  // Indices for input/output ports
  const int lane_state_input_port_index_{};
  const int pose_input_port_index_{};
  const int velocity_input_port_index_{};
  const int velocity_output_port_index_{};
  maliput::api::RoadNetwork* road_network_;
};

}  // namespace delphyne
