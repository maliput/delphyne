// Copyright 2018 Toyota Research Institute

#include "systems/trajectory_follower.h"

#include <drake/common/default_scalars.h>

namespace delphyne {

using drake::multibody::SpatialVelocity;
using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;

template <typename T>
TrajectoryFollower<T>::TrajectoryFollower(const Trajectory& trajectory,
                                          double sampling_time_sec)
    : drake::systems::LeafSystem<T>(
          drake::systems::SystemTypeTag<TrajectoryFollower>{}),
      trajectory_(trajectory) {
  this->DeclarePeriodicUnrestrictedUpdate(sampling_time_sec, 0.);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcStateOutput);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcPoseOutput);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcVelocityOutput);
}

template <typename T>
void TrajectoryFollower<T>::CalcStateOutput(
    const drake::systems::Context<T>& context,
    SimpleCarState<T>* output_vector) const {
  const PoseVelocity values = GetValues(context);
  output_vector->set_x(T{values.pose3().x()});
  output_vector->set_y(T{values.pose3().y()});
  output_vector->set_heading(T{values.pose3().z()});
  output_vector->set_velocity(T{values.speed()});
}

template <typename T>
void TrajectoryFollower<T>::CalcPoseOutput(
    const drake::systems::Context<T>& context, PoseVector<T>* pose) const {
  const PoseVelocity values = GetValues(context);
  pose->set_translation(Eigen::Translation<T, 3>{values.translation()});
  const Eigen::Quaternion<double>& q =
      drake::math::RotationMatrix<double>(values.rotation()).ToQuaternion();
  pose->set_rotation(Eigen::Quaternion<T>{q});
}

template <typename T>
void TrajectoryFollower<T>::CalcVelocityOutput(
    const drake::systems::Context<T>& context,
    FrameVelocity<T>* velocity) const {
  const PoseVelocity values = GetValues(context);
  const Eigen::Vector3d& v = values.velocity().translational();
  const Eigen::Vector3d& w = values.velocity().rotational();
  velocity->set_velocity(
      SpatialVelocity<T>{drake::Vector3<T>{w}, drake::Vector3<T>{v}});
}

template <typename T>
PoseVelocity TrajectoryFollower<T>::GetValues(
    const drake::systems::Context<T>& context) const {
  return trajectory_.value(drake::ExtractDoubleOrThrow(context.get_time()));
}

}  // namespace delphyne

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::delphyne::TrajectoryFollower)
