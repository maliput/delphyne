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

#include "backend/frame_pose_aggregator.h"

#include <delphyne/macros.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/systems/rendering/pose_vector.h>

namespace delphyne {

template <typename T>
FramePoseAggregator<T>::FramePoseAggregator() {
  // Declare the output port and provide an allocator for a FramePoseVector of
  // length equal to the concatenation of all inputs.
  this->DeclareAbstractOutputPort(&FramePoseAggregator::MakeFramePoseVector, &FramePoseAggregator::CalcFramePoseVector);
}

template <typename T>
const drake::systems::InputPort<T>& FramePoseAggregator<T>::DeclareInput(const drake::geometry::FrameId& frame_id) {
  // Make sure no other input was declared with the same frame ID.
  DELPHYNE_VALIDATE(std::find(frame_ids_.begin(), frame_ids_.end(), frame_id) == frame_ids_.end(), std::runtime_error,
                    "Duplicate input with same frame ID");
  const drake::systems::InputPort<T>& pose_port =
      this->DeclareVectorInputPort(drake::systems::rendering::PoseVector<T>());
  frame_ids_.push_back(frame_id);
  return pose_port;
}

template <typename T>
drake::geometry::FramePoseVector<T> FramePoseAggregator<T>::MakeFramePoseVector() const {
  return drake::geometry::FramePoseVector<T>();
}

template <typename T>
void FramePoseAggregator<T>::CalcFramePoseVector(const drake::systems::Context<T>& context,
                                                 drake::geometry::FramePoseVector<T>* output) const {
  using drake::systems::rendering::PoseVector;
  output->clear();
  const int num_ports = this->num_input_ports();
  for (int port_index = 0; port_index < num_ports; ++port_index) {
    const PoseVector<T>* input_pose = this->template EvalVectorInput<PoseVector>(context, port_index);
    output->set_value(frame_ids_[port_index], input_pose->get_transform());
  }
}

// TODO(hidmic): Extend support for non-double types when SceneGraph supports
// them.
template class FramePoseAggregator<double>;

}  // namespace delphyne
