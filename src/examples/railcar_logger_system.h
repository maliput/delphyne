// Copyright 2019 Toyota Research Institute
#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "gen/rail_follower_state.h"
#include "systems/lane_direction.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace delphyne {

/// System thought to log all the MaliputRailcar outputs.
class RailcarLoggerSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RailcarLoggerSystem)

  RailcarLoggerSystem();

  /// Returns the port to the individual input ports.
  const drake::systems::InputPort<double>& ego_pose_input() const;
  const drake::systems::InputPort<double>& ego_velocity_input() const;
  const drake::systems::InputPort<double>& car_state_input() const;
  const drake::systems::InputPort<double>& lane_state_input() const;

 private:
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>*) const override;

  void log_ego_pose(
      const drake::systems::rendering::PoseVector<double>* const ego_pose,
      const double& time) const;

  void log_ego_velocity(
      const drake::systems::rendering::FrameVelocity<double>* const
        ego_velocity,
      const double& time) const;

  void log_car_state(
      const delphyne::RailFollowerState<double>* const car_state,
      const double& time) const;

  void log_lane_state(
      const LaneDirection* const lane_direction,
      const double& time) const;

  const int ego_pose_index_{};
  const int ego_velocity_index_{};
  const int car_state_index_{};
  const int lane_state_index_{};
};

}  // namespace delphyne
