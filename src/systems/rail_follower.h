// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <vector>

#include <drake/automotive/maliput/api/lane.h>
#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/system_symbolic_inspector.h>
#include <drake/systems/rendering/frame_velocity.h>
#include <drake/systems/rendering/pose_vector.h>

#include "gen/rail_follower_params.h"
#include "gen/rail_follower_state.h"
#include "gen/simple_car_state.h"
#include "systems/lane_direction.h"

namespace delphyne {

/// RailFollower models an entity that follows a maliput::api::Lane as if it
/// were on rails and neglecting all physics.
///
/// Parameters:
///   * See RailFollowerParams.
///
/// Continuous State:
///   * See RailFollowerState.
///
/// Abstract state:
///   * See LaneDirection.
///
/// <B>Input Port Accessors:</B>
///
///   - command_input(): Contains the desired acceleration. This port
///     contains a drake::systems::BasicVector of size 1. It is optional
///     in that it need not be connected. When it is unconnected, the
///     modelled entity will travel at its initial velocity, which is
///     specified in RailFollowerParams.
///
/// <B>Output Port Accessors:</B>
///
///   - state_output(): Contains this system's state vector. See
///     RailFollowerState.
///
///   - lane_state_output(): Contains this system's lane direction state. See
///     LaneDirection.
///
///   - pose_output(): Contains PoseVector `X_WC`, where `C` is the car frame
///     and `W` is the world frame.
///
///   - velocity_output(): Contains FrameVelocity `V_WC_W`, where `C` is the car
///     frame and `W` is the world frame. Currently the rotational component is
///     always zero, see #5751.
///
/// @tparam T must support certain arithmetic operations;
/// for details, see delphyne::Symbolic.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
template <typename T>
class RailFollower final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RailFollower)
  /// Defines a distance that is "close enough" to the end of a lane for the
  /// modelled entity to transition to an ongoing branch. The primary
  /// constraint on the selection of this variable is the application's
  /// degree of sensitivity to position state discontinuity when the
  /// modelled entity "jumps" from its current lane to a lane in an
  /// ongoing branch. A smaller value results in a
  /// smaller spatial discontinuity. If this value is zero, the spatial
  /// discontinuity will be zero. However, it will trigger the use of
  /// kTimeEpsilon, which results in a temporal discontinuity.
  static constexpr double kLaneEndEpsilon{1e-12};

  /// Defines a time interval that is used to ensure a desired update time is
  /// always greater than (i.e., after) the current time. Despite the spatial
  /// window provided by kLaneEndEpsilon, it is still possible for the vehicle
  /// to end up precisely at the end of its current lane (e.g., it could be
  /// initialized in this state). In this scenario, the next update time
  /// will be equal to the current time. The integrator, however, requires
  /// that the next update time be strictly after the current time, which
  /// is when this constant is used. The primary constraint on the selection
  /// of this constant is the application's sensitivity to the modelled
  /// entity being "late" in its transition to an ongoing branch once it
  /// is at the end of its current lane. The smaller this value, the less
  /// "late" the transition will occur. This value cannot be zero since
  /// that will violate the integrator's need for the next update time to
  /// be strictly after the current time.
  static constexpr double kTimeEpsilon{1e-12};

  /// The constructor.
  ///
  /// @param initial_lane_direction The initial lane and direction of travel.
  ///
  explicit RailFollower(const LaneDirection& initial_lane_direction);

  /// Constructor that initialises all variables in the system that will be
  /// placed on the context. This includes the continuous state, parameters
  /// and non-continuous state variables.
  ///
  /// @param[in] initial_lane_direction The initial lane and direction of
  /// travel.
  /// @param[in] initial_context_state The continuous state variables
  /// @param[in] initial_context_parameters The parameters
  RailFollower(const LaneDirection& initial_lane_direction,
               const RailFollowerState<T>& initial_context_state,
               const RailFollowerParams<T>& initial_context_parameters);

  /// Returns a mutable reference to the parameters in the given @p context.
  RailFollowerParams<T>& get_mutable_parameters(
      drake::systems::Context<T>* context) const;

  /// Getter methods for input and output ports.
  /// @{
  const drake::systems::InputPort<T>& command_input() const;
  const drake::systems::OutputPort<T>& state_output() const;
  const drake::systems::OutputPort<T>& simple_car_state_output() const;
  const drake::systems::OutputPort<T>& lane_state_output() const;
  const drake::systems::OutputPort<T>& pose_output() const;
  const drake::systems::OutputPort<T>& velocity_output() const;
  /// @}

 private:
  // System<T> overrides.
  void DoCalcTimeDerivatives(
      const drake::systems::Context<T>& context,
      drake::systems::ContinuousState<T>* derivatives) const override;

  // LeafSystem<T> overrides.
  drake::optional<bool> DoHasDirectFeedthrough(int, int) const override;
  void DoCalcNextUpdateTime(const drake::systems::Context<T>& context,
                            drake::systems::CompositeEventCollection<T>*,
                            T* time) const override;
  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<T>& context,
      const std::vector<const drake::systems::UnrestrictedUpdateEvent<T>*>&,
      drake::systems::State<T>* state) const override;

  void CalcSimpleCarStateOutput(const drake::systems::Context<T>& context,
                                SimpleCarState<T>* output) const;

  void CalcStateOutput(const drake::systems::Context<T>& context,
                       RailFollowerState<T>* output) const;

  void CalcLaneOutput(const drake::systems::Context<T>& context,
                      LaneDirection* output) const;

  void CalcPose(const drake::systems::Context<T>& context,
                drake::systems::rendering::PoseVector<T>* pose) const;

  void CalcVelocity(const drake::systems::Context<T>& context,
                    drake::systems::rendering::FrameVelocity<T>* pose) const;

  void ImplCalcTimeDerivatives(const RailFollowerParams<T>& params,
                               const RailFollowerState<T>& state,
                               const LaneDirection& lane_direction,
                               const drake::systems::BasicVector<T>& input,
                               RailFollowerState<T>* rates) const;

  void ImplCalcTimeDerivativesDouble(const RailFollowerParams<double>& params,
                                     const RailFollowerState<double>& state,
                                     RailFollowerState<double>* rates) const;

  // Calculates the vehicle's `r` coordinate based on whether it's traveling
  // with or against `s` in the current lane relative to the initial lane.
  T CalcR(const RailFollowerParams<T>& params,
          const LaneDirection& lane_direction) const;

  /****************************************
   * Context Accessors
   ***************************************/
  const RailFollowerState<T>& get_rail_follower_state(
      const drake::systems::Context<T>& context) const;

  const RailFollowerParams<T>& get_rail_follower_parameters(
      const drake::systems::Context<T>& context) const;

  const LaneDirection& get_lane_direction(
      const drake::systems::Context<T>& context) const;

  /********************
   * Initial Values
   *******************/
  const LaneDirection initial_lane_direction_{};

  /********************
   * System Indices
   *******************/
  int command_input_port_index_{};
  int simple_car_state_output_port_index_{};
  int state_output_port_index_{};
  int lane_state_output_port_index_{};
  int pose_output_port_index_{};
  int velocity_output_port_index_{};
  int rail_follower_params_context_index_{};
  int lane_direction_context_index_{};
};

}  // namespace delphyne
