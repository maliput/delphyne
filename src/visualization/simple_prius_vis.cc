/**
 * @file src/visualization/simple_prius_vis.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "visualization/simple_prius_vis.h"

#include <Eigen/Geometry>

#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/joints/roll_pitch_yaw_floating_joint.h>
#include <drake/multibody/kinematics_cache.h>
#include <drake/multibody/parsers/parser_common.h>
#include <drake/multibody/parsers/sdf_parser.h>
#include <drake/multibody/rigid_body_plant/create_load_robot_message.h>

#include "delphyne/macros.h"

using std::unique_ptr;
using std::vector;

namespace delphyne {

using drake::multibody::joints::kRollPitchYaw;
using drake::systems::rendering::PoseBundle;
using drake::Vector3;
using drake::VectorX;

template <typename T>
constexpr double SimplePriusVis<T>::kVisOffset;

template <typename T>
SimplePriusVis<T>::SimplePriusVis(int id, const std::string& name)
    : CarVis<T>(id, name), tree_(new RigidBodyTree<T>()) {
  const char* delphyne_resource_root = std::getenv("DELPHYNE_RESOURCE_ROOT");

  DELPHYNE_DEMAND(delphyne_resource_root != NULL);

  std::stringstream sdf_filename;
  sdf_filename << delphyne_resource_root << "/media/prius/simple_prius.sdf";

  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      sdf_filename.str(), kRollPitchYaw, tree_.get());

  // Verifies that the model instance within tree_ meets this method's
  // requirements. See the class description for more details.
  DRAKE_DEMAND(tree_->get_num_model_instances() == 1);
  const std::vector<int> base_body_indices = tree_->FindBaseBodies();
  DRAKE_DEMAND(base_body_indices.size() == 1);
  const RigidBody<T>& body = tree_->get_body(base_body_indices.at(0));
  const RollPitchYawFloatingJoint* rpy_joint =
      dynamic_cast<const RollPitchYawFloatingJoint*>(&body.getJoint());
  DRAKE_DEMAND(rpy_joint != nullptr);

  drake::lcmt_viewer_load_robot load_message =
      drake::multibody::CreateLoadRobotMessage<T>(*tree_);
  for (const auto& link : load_message.link) {
    if (link.name != RigidBodyTreeConstants::kWorldName) {
      vis_elements_.push_back(link);
      vis_elements_.back().robot_num = id;
    }
  }
}

template <typename T>
const vector<lcmt_viewer_link_data>& SimplePriusVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
drake::systems::rendering::PoseBundle<T> SimplePriusVis<T>::CalcPoses(
    const Isometry3<T>& X_WM) const {
  // Computes X_MV, the transform from the visualization's frame to the model's
  // frame. The 'V' in the variable name stands for "visualization". This is
  // necessary because the model frame's origin is centered at the midpoint of
  // the vehicle's rear axle whereas the visualization frame's origin is
  // centered in the middle of a body called "chassis_floor". The axes of the
  // two frames are parallel with each other. However, the distance between the
  // origins of the two frames is 1.40948 m along the model's x-axis.
  const Isometry3<T> X_MV(Eigen::Translation<T, 3>(
      T(1.40948) /* x offset */, T(0) /* y offset */, T(0) /* z offset */));
  const Isometry3<T> X_WV = X_WM * X_MV;
  const auto rotation = X_WV.linear();
  const auto transform = X_WV.translation();
  Vector3<T> rpy = rotation.eulerAngles(2, 1, 0);
  VectorX<T> q = VectorX<T>::Zero(tree_->get_num_positions());
  q(0) = transform.x();
  q(1) = transform.y();
  q(2) = transform.z();
  q(3) = rpy(2);
  q(4) = rpy(1);
  q(5) = rpy(0);
  const KinematicsCache<T> cache = tree_->doKinematics(q);

  PoseBundle<T> result(this->num_poses());
  int result_index{0};
  for (int i = 0; i < tree_->get_num_bodies(); ++i) {
    if (i != RigidBodyTreeConstants::kWorldBodyIndex) {
      const RigidBody<T>& body = tree_->get_body(i);
      const drake::Isometry3<T> X_WB =
          tree_->CalcBodyPoseInWorldFrame(cache, body);
      result.set_pose(result_index, X_WB);
      result.set_name(result_index, body.get_name());
      result.set_model_instance_id(result_index, this->id());
      ++result_index;
    }
  }
  DRAKE_ASSERT(result_index == this->num_poses());
  return result;
}

// These instantiations must match the API documentation in
// prius_vis.h.
template class SimplePriusVis<double>;

}  // namespace delphyne
