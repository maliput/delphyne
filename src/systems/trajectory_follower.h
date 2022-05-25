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

#include <Eigen/Geometry>
#include <drake/common/drake_copyable.h>
#include <drake/common/extract_double.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/rendering/frame_velocity.h>
#include <drake/systems/rendering/pose_vector.h>

#include "gen/simple_car_state.h"
#include "systems/trajectory.h"

namespace delphyne {

/// TrajectoryFollower simply moves along a pre-established trajectory.
///
/// Note that, when T = AutoDiffXd, the AutoDiffXd derivatives for each element
/// of the the outputs are empty.
///
/// output port 0: A SimpleCarState containing:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, positive-left-turn.
/// * speed: s = √{ẋ² + ẏ²}
///   (OutputPort getter: state_output())
///
/// output port 1: A PoseVector containing X_WA, where A is the agent's
/// reference frame.
///   (OutputPort getter: pose_output())
///
/// output port 2: A FrameVelocity containing the spatial velocity V_WA, where A
/// is the agent's reference frame.
///   (OutputPort getter: velocity_output())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class TrajectoryFollower final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryFollower)

  /// Constructs a TrajectoryFollower system that traces a given Trajectory.
  ///
  /// @param trajectory a Trajectory containing the trajectory.
  /// @param sampling_time_sec the requested sampling time (in sec) for this
  /// system. Default: 0.01.
  TrajectoryFollower(const Trajectory& trajectory, double sampling_time_sec = 0.01);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit TrajectoryFollower(const TrajectoryFollower<U>& other) : TrajectoryFollower<T>(other.trajectory_) {}

  /// @name Accessors for the outputs, as enumerated in the class documentation.
  /// @{
  const drake::systems::OutputPort<T>& state_output() const { return this->get_output_port(0); }
  const drake::systems::OutputPort<T>& pose_output() const { return this->get_output_port(1); }
  const drake::systems::OutputPort<T>& velocity_output() const { return this->get_output_port(2); }
  /// @}

 private:
  // Converts a PoseVelocity, evaluated at the current time, into a
  // SimpleCarState output.
  void CalcStateOutput(const drake::systems::Context<T>& context, SimpleCarState<T>* output_vector) const;

  // Converts a PoseVelocity, evaluated at the current time, into a PoseVector
  // output.
  void CalcPoseOutput(const drake::systems::Context<T>& context, drake::systems::rendering::PoseVector<T>* pose) const;

  // Converts a PoseVelocity, evaluated at the current time, into a
  // FrameVelocity output.
  void CalcVelocityOutput(const drake::systems::Context<T>& context,
                          drake::systems::rendering::FrameVelocity<T>* velocity) const;

  // Extracts the PoseVelocity value at the current time, as provided in
  // Context.
  PoseVelocity GetValues(const drake::systems::Context<T>& context) const;

  // Allow different specializations to access each other's private data.
  template <typename>
  friend class TrajectoryFollower;

  const Trajectory trajectory_;
};

}  // namespace delphyne
