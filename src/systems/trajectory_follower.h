// Copyright 2018 Toyota Research Institute

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
  /// system.  @default 0.01.
  TrajectoryFollower(const Trajectory& trajectory, double sampling_time_sec = 0.01);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
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
