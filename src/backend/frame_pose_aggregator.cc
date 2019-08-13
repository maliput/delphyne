// Copyright 2018 Toyota Research Institute

#include "backend/frame_pose_aggregator.h"

#include <delphyne/macros.h>

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
    output->set_value(frame_ids_[port_index], input_pose->get_isometry());
  }
}

// TODO(hidmic): Extend support for non-double types when SceneGraph supports
// them.
template class FramePoseAggregator<double>;

}  // namespace delphyne
