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

#include <map>
#include <memory>

#include <drake/common/drake_copyable.h>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <drake/systems/rendering/pose_vector.h>

#include "visualization/car_vis.h"

namespace delphyne {

/// CarVisApplicator takes as input a PoseVector containing vehicle poses. For
/// each vehicle, it outputs the poses of all visual geometries associated with
/// the vehicle's visualization.
///
/// Prior to instantiating this system's drake::systems::Context and
/// drake::systems::SystemOutput, a CarVis object must be provided for each
/// vehicle in
/// the simulation using AddCarVis().
///
/// This system is stateless and is direct feed-through.
///
/// Input port getters:
///  - get_car_poses_input_port() - Contains a PoseBundle of every vehicle's
///    pose in the world frame (i.e., `X_WM_W` where `W` stands for "world" and
///    `M` stands for "Model"). The vehicle IDs and names contained within this
///    PoseBundle must match the IDs and names contained within the CarVis
///    objects that were supplied via calls to AddCarVis().
///
/// Output port getters:
///  - get_visual_geometry_poses_output_port() - Contains a PoseBundle of visual
///    geometry poses in the world frame.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
template <typename T>
class CarVisApplicator : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CarVisApplicator)

  CarVisApplicator();
  ~CarVisApplicator() override {}

  /// Returns the input port that contains the vehicle poses in
  /// the form of a PoseBundle.
  const drake::systems::InputPort<T>& get_car_poses_input_port() const;

  /// Returns the output port that contains the visual geometry
  /// poses of all vehicle visualizations.
  const drake::systems::OutputPort<T>& get_visual_geometry_poses_output_port() const;

  /// Adds a CarVis object for a vehicle. The ID returned by CarVis::id() must
  /// be unique among the CarVis objects added to this method. A
  /// std::runtime_error is thrown if the provided CarVis object's ID is a
  /// duplicate of a previously provided CarVis object's ID.
  ///
  /// @pre The context for this system has not been created.
  void AddCarVis(std::unique_ptr<CarVis<T>> vis);

  /// Returns an lcmt_viewer_load_robot message containing the geometries of the
  /// bodies being visualized.
  drake::lcmt_viewer_load_robot get_load_robot_message() const;

  /// Returns the number vehicles being visualized.
  int num_cars() const { return static_cast<int>(visualizers_.size()); }

  /// Returns the total number of poses of bodies being visualized.
  int num_vis_poses() const;

 private:
  drake::systems::rendering::PoseBundle<T> MakePoseBundleOutput() const;

  void CalcPoseBundleOutput(const drake::systems::Context<T>& context,
                            drake::systems::rendering::PoseBundle<T>* output) const;

  // The key is the car ID.
  std::map<int, std::unique_ptr<const CarVis<T>>> visualizers_;

  // The key is the car ID and the value is the starting index within the output
  // PoseBundle.
  mutable std::map<int, int> starting_indices_;

  int input_port_index_{};
  int output_port_index_{};
  bool context_allocated_{false};
};

}  // namespace delphyne
