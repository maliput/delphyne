// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <drake/automotive/maliput/api/lane_data.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <drake/systems/rendering/pose_vector.h>

#include "gen/idm_planner_parameters.h"
#include "systems/calc_ongoing_road_position.h"
#include "systems/idm_planner.h"
#include "systems/traffic_pose_selector.h"

namespace delphyne {

/// IDMController implements the IDM (Intelligent Driver Model) planner,
/// computed based only on the nearest car ahead.  See IdmPlanner and
/// PoseSelector for details.  The output of this block is an acceleration value
/// passed as a command to the vehicle.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: PoseVector for the ego car.
///   (InputPort getter: ego_pose_input())
///
/// Input Port 1: FrameVelocity of the ego car.
///   (InputPort getter: ego_velocity_input())
///
/// Input Port 2: PoseBundle for the traffic cars, possibly inclusive of the ego
///   car's pose.
///   (InputPort getter: traffic_input())
///
/// Output Port 0: A BasicVector containing the acceleration request.
///   (OutputPort getter: acceleration_output())
///
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_controllers
template <typename T>
class IDMController : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IDMController)

  /// Constructor.
  /// @param road The pre-defined RoadGeometry.
  /// @param path_or_branches If ScanStrategy::kBranches, performs IDM
  /// computations using vehicles detected in confluent branches; if
  /// ScanStrategy::kPath, limits to vehicles on the default path.  See
  /// documentation for PoseSelector::FindSingleClosestPose().
  /// @param road_position_strategy Determines whether or not to cache
  /// RoadPosition. See `calc_ongoing_road_position.h`.
  /// @param period_sec The update period to use if road_position_strategy ==
  /// RoadPositionStrategy::kCache.
  IDMController(const drake::maliput::api::RoadGeometry& road,
                ScanStrategy path_or_branches,
                RoadPositionStrategy road_position_strategy, double period_sec);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit IDMController(const IDMController<U>& other)
      : IDMController<T>(other.road_, other.path_or_branches_,
                         other.road_position_strategy_, other.period_sec_) {}

  ~IDMController() override;

  /// See the class description for details on the following input ports.
  /// @{
  const drake::systems::InputPort<T>& ego_pose_input() const;
  const drake::systems::InputPort<T>& ego_velocity_input() const;
  const drake::systems::InputPort<T>& traffic_input() const;
  const drake::systems::OutputPort<T>& acceleration_output() const;
  /// @}

 protected:
  const drake::maliput::api::RoadGeometry& road() const { return road_; }
  int ego_pose_index() const { return ego_pose_index_; }
  int ego_velocity_index() const { return ego_velocity_index_; }
  int traffic_index() const { return traffic_index_; }
  int acceleration_index() const { return acceleration_index_; }

  void ImplCalcAcceleration(
      const drake::systems::rendering::PoseVector<T>& ego_pose,
      const drake::systems::rendering::FrameVelocity<T>& ego_velocity,
      const drake::systems::rendering::PoseBundle<T>& traffic_poses,
      const IdmPlannerParameters<T>& idm_params,
      const drake::maliput::api::RoadPosition& ego_rp,
      drake::systems::BasicVector<T>* command) const;

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<T>& context,
      const std::vector<const drake::systems::UnrestrictedUpdateEvent<T>*>&,
      drake::systems::State<T>* state) const override;

 private:
  // Allow different specializations to access each other's private data.
  template <typename>
  friend class IDMController;

  // Converts @p pose into RoadPosition.
  const drake::maliput::api::RoadPosition GetRoadPosition(
      const drake::Isometry3<T>& pose) const;

  void CalcAcceleration(const drake::systems::Context<T>& context,
                        drake::systems::BasicVector<T>* accel_output) const;

  const drake::maliput::api::RoadGeometry& road_;
  const ScanStrategy path_or_branches_{};
  const RoadPositionStrategy road_position_strategy_{};
  const double period_sec_{};

  // Indices for the input / output ports.
  const int ego_pose_index_{};
  const int ego_velocity_index_{};
  const int traffic_index_{};
  const int acceleration_index_{};
};

}  // namespace delphyne

namespace drake {
namespace systems {
namespace scalar_conversion {
// Disable symbolic support, because we use ExtractDoubleOrThrow.
template <>
struct Traits<::delphyne::IDMController> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
