// Copyright 2018 Toyota Research Institute

#pragma once

#include <memory>

#include <Eigen/Geometry>

#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/scalar_conversion_traits.h>
#include <drake/systems/rendering/pose_vector.h>

#include "gen/pure_pursuit_params.h"
#include "gen/simple_car_params.h"
#include "systems/lane_direction.h"

namespace delphyne {

/// PurePursuitController implements a pure pursuit controller.  See PurePursuit
/// for details on the approach.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: a LaneDirection representing the requested lane and direction
///   of travel.
///   (InputPort getter: lane_input())
///
/// Input Port 1: PoseVector for the ego car.
///   (InputPort getter: ego_pose_input())
///
/// Output Port 0: A BasicVector of size one with the commanded steering angle.
///   (OutputPort getter: steering_command_output())
///
/// @ingroup automotive_controllers
template <typename T>
class PurePursuitController : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuitController)

  /// Constructor.
  PurePursuitController();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit PurePursuitController(const PurePursuitController<U>&) : PurePursuitController<T>() {}

  ~PurePursuitController() override;

  /// Returns the port to the individual input/output ports.
  const drake::systems::InputPort<T>& lane_input() const;
  const drake::systems::InputPort<T>& ego_pose_input() const;
  const drake::systems::OutputPort<T>& steering_command_output() const;

 private:
  void OutputSteeringCommand(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;

  void CalcSteeringCommand(const PurePursuitParams<T>& pp_params, const SimpleCarParams<T>& car_params,
                           const LaneDirection& lane_direction,
                           const drake::systems::rendering::PoseVector<T>& ego_pose,
                           drake::systems::BasicVector<T>* command) const;

  // Indices for the input / output ports.
  const int lane_index_{};
  const int ego_pose_index_{};
  const int steering_command_index_{};
};

}  // namespace delphyne

namespace drake {
namespace systems {
namespace scalar_conversion {
// Disables symbolic support, because maliput's LanePosition <-> InertialPosition
// conversion (used in pure_pursuit.cc) is not symbolic-supported.
template <>
struct Traits<::delphyne::PurePursuitController> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
