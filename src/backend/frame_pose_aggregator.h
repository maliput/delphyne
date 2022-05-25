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

#pragma once

#include <vector>

#include <drake/common/drake_copyable.h>
#include <drake/geometry/frame_kinematics_vector.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/leaf_system.h>

namespace delphyne {

/// A system that aggregates drake::systems::rendering::PoseVector inputs
/// into a single drake::geometry::FramePoseVector output, each one associated
/// to a specific frame ID. Akin in functionality to
/// drake::systems::rendering::PoseAggregator.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class FramePoseAggregator : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FramePoseAggregator)

  FramePoseAggregator();

  /// Declares a pose input port, associated with the given @p frame_id.
  /// @return The input drake::systems::rendering::PoseVector port
  /// descriptor.
  const drake::systems::InputPort<T>& DeclareInput(const drake::geometry::FrameId& frame_id);

 private:
  // Returns a new drake::geometry::FramePoseVector instance to
  // populate.
  drake::geometry::FramePoseVector<T> MakeFramePoseVector() const;

  // Builds the outgoing drake::geometry::FramePoseVector based on
  // declared drake::systems::rendering::PoseVector inputs.
  void CalcFramePoseVector(const drake::systems::Context<T>& context,
                           drake::geometry::FramePoseVector<T>* output) const;

  // Aggregator inputs' associated frame IDs.
  std::vector<drake::geometry::FrameId> frame_ids_{};
};

}  // namespace delphyne
