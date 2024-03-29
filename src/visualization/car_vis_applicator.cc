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
#include "visualization/car_vis_applicator.h"

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <drake/common/drake_assert.h>
#include <drake/common/value.h>
#include <drake/math/rigid_transform.h>

using std::unique_ptr;
using std::vector;

using drake::Value;
using drake::math::RigidTransform;
using drake::systems::Context;
using drake::systems::rendering::PoseBundle;
using drake::systems::rendering::PoseVector;

namespace delphyne {

template <typename T>
CarVisApplicator<T>::CarVisApplicator() {
  input_port_index_ = drake::systems::LeafSystem<T>::DeclareAbstractInputPort(Value<PoseBundle<T>>(0)).get_index();
  output_port_index_ = drake::systems::LeafSystem<T>::DeclareAbstractOutputPort(&CarVisApplicator::MakePoseBundleOutput,
                                                                                &CarVisApplicator::CalcPoseBundleOutput)
                           .get_index();
}

template <typename T>
const drake::systems::InputPort<T>& CarVisApplicator<T>::get_car_poses_input_port() const {
  return drake::systems::System<T>::get_input_port(input_port_index_);
}

template <typename T>
const drake::systems::OutputPort<T>& CarVisApplicator<T>::get_visual_geometry_poses_output_port() const {
  return drake::systems::System<T>::get_output_port(output_port_index_);
}

template <typename T>
void CarVisApplicator<T>::AddCarVis(std::unique_ptr<CarVis<T>> vis) {
  const int id = vis->id();
  if (visualizers_.find(id) != visualizers_.end()) {
    throw std::runtime_error(
        "CarVisApplicator::AddCarVis(): Attempted to add "
        "CarVis with duplicate ID of " +
        std::to_string(id) + ".");
  }
  visualizers_[id] = std::move(vis);
}

template <typename T>
drake::lcmt_viewer_load_robot CarVisApplicator<T>::get_load_robot_message() const {
  drake::lcmt_viewer_load_robot result;
  for (const auto& visualizer : visualizers_) {
    const std::vector<drake::lcmt_viewer_link_data>& vis_elements = visualizer.second->GetVisElements();
    for (const auto& vis_element : vis_elements) {
      result.link.push_back(vis_element);
    }
  }
  result.num_links = result.link.size();
  return result;
}

template <typename T>
void CarVisApplicator<T>::CalcPoseBundleOutput(const drake::systems::Context<T>& context,
                                               PoseBundle<T>* visualization_poses) const {
  // Obtains the input and output.
  const PoseBundle<T>& vehicle_poses =
      drake::systems::System<T>::EvalAbstractInput(context, input_port_index_)->template get_value<PoseBundle<T>>();
  DRAKE_ASSERT(vehicle_poses.get_num_poses() == num_cars());

  if (vehicle_poses.get_num_poses() != static_cast<int>(visualizers_.size())) {
    throw std::runtime_error(
        "CarVisApplicator::DoCalcOutput(): Input "
        "PoseBundle has " +
        std::to_string(vehicle_poses.get_num_poses()) + " poses. Expected " + std::to_string(visualizers_.size()) +
        ".");
  }

  for (int i = 0; i < vehicle_poses.get_num_poses(); ++i) {
    const int id = vehicle_poses.get_model_instance_id(i);
    const auto vis_iterator = visualizers_.find(id);
    const auto index_iterator = starting_indices_.find(id);
    if (vis_iterator == visualizers_.end()) {
      throw std::runtime_error(
          "CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid ID of " +
          std::to_string(id) +
          ". No CarVis "
          "matches this ID.");
    }
    if (index_iterator == starting_indices_.end()) {
      throw std::runtime_error(
          "CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid ID of " +
          std::to_string(id) +
          ". No "
          "starting index matches this ID.");
    }
    const auto& car_vis = vis_iterator->second;
    int index = index_iterator->second;
    const std::string& name = vehicle_poses.get_name(i);
    const std::string& expected_name = car_vis->name();
    if (name != expected_name) {
      throw std::runtime_error(
          "CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid model name with ID " +
          std::to_string(id) + ". Expected \"" + expected_name + "\" but got \"" + name + "\".");
    }
    const drake::Isometry3<T>& root_pose = vehicle_poses.get_transform(i).GetAsIsometry3();
    const PoseBundle<T> model_vis_poses = car_vis->CalcPoses(root_pose);
    for (int j = 0; j < model_vis_poses.get_num_poses(); ++j) {
      visualization_poses->set_transform(index, model_vis_poses.get_transform(j));
      index++;
    }
  }
}

template <typename T>
PoseBundle<T> CarVisApplicator<T>::MakePoseBundleOutput() const {
  PoseBundle<T> pose_bundle(num_vis_poses());
  int index{0};
  for (const auto& v : visualizers_) {
    const int id = v.first;
    const auto& car_vis = v.second;
    // Storing the index in a member variable is OK since it's done prior to
    // the start of simulation and remains constant throughout the simulation.
    starting_indices_[id] = index;
    const std::vector<drake::lcmt_viewer_link_data>& link_data = car_vis->GetVisElements();
    for (const auto& data : link_data) {
      DRAKE_DEMAND(index < num_vis_poses());
      pose_bundle.set_name(index, data.name);
      pose_bundle.set_model_instance_id(index, id);
      ++index;
    }
  }
  return pose_bundle;
}

template <typename T>
int CarVisApplicator<T>::num_vis_poses() const {
  int result{0};
  for (const auto& v : visualizers_) {
    result += v.second->num_poses();
  }
  return result;
}

// These instantiations must match the API documentation in
// car_vis_applicator.h.
template class CarVisApplicator<double>;

}  // namespace delphyne
