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

#include "systems/trajectory_follower.h"

#include <drake/common/default_scalars.h>

namespace delphyne {

using drake::multibody::SpatialVelocity;
using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;

template <typename T>
TrajectoryFollower<T>::TrajectoryFollower(const Trajectory& trajectory, double sampling_time_sec)
    : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<TrajectoryFollower>{}), trajectory_(trajectory) {
  this->DeclarePeriodicUnrestrictedUpdate(sampling_time_sec, 0.);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcStateOutput);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcPoseOutput);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcVelocityOutput);
}

template <typename T>
void TrajectoryFollower<T>::CalcStateOutput(const drake::systems::Context<T>& context,
                                            SimpleCarState<T>* output_vector) const {
  const PoseVelocity values = GetValues(context);
  output_vector->set_x(T{values.pose3().x()});
  output_vector->set_y(T{values.pose3().y()});
  output_vector->set_heading(T{values.pose3().z()});
  output_vector->set_velocity(T{values.speed()});
}

template <typename T>
void TrajectoryFollower<T>::CalcPoseOutput(const drake::systems::Context<T>& context, PoseVector<T>* pose) const {
  const PoseVelocity values = GetValues(context);
  pose->set_translation(Eigen::Translation<T, 3>{values.translation()});
  const Eigen::Quaternion<double>& q = drake::math::RotationMatrix<double>(values.rotation()).ToQuaternion();
  pose->set_rotation(Eigen::Quaternion<T>{q});
}

template <typename T>
void TrajectoryFollower<T>::CalcVelocityOutput(const drake::systems::Context<T>& context,
                                               FrameVelocity<T>* velocity) const {
  const PoseVelocity values = GetValues(context);
  const Eigen::Vector3d& v = values.velocity().translational();
  const Eigen::Vector3d& w = values.velocity().rotational();
  velocity->set_velocity(SpatialVelocity<T>{drake::Vector3<T>{w}, drake::Vector3<T>{v}});
}

template <typename T>
PoseVelocity TrajectoryFollower<T>::GetValues(const drake::systems::Context<T>& context) const {
  return trajectory_.value(drake::ExtractDoubleOrThrow(context.get_time()));
}

}  // namespace delphyne

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::delphyne::TrajectoryFollower)
