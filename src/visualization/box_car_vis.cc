// Copyright 2018 Toyota Research Institute
#include "visualization/box_car_vis.h"

#include <Eigen/Dense>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>

using std::vector;

namespace delphyne {

using drake::math::RigidTransform;
using drake::systems::rendering::PoseBundle;

namespace {

// Creates a drake::lcmt_viewer_geometry_data for a box.
//
// Copied and refactored from
// https://github.com/RobotLocomotion/drake/blob/v0.22.0/attic/systems/rendering/drake_visualizer_client.cc#L9-L87
// Specifically, it is restricted to boxes.
//
// @param box_size A 3D vector with the box size for each coordinate.
// @param local_transform A 3D Isometry containing the box local transformation.
// @param material A 4D vector with the box material.
// @return A drake::lcmt_viewer_geometry_data representing a box.
drake::lcmt_viewer_geometry_data MakeBoxGeometryData(const Eigen::Vector3d& box_size,
                                                     const Eigen::Isometry3d& local_transform,
                                                     const Eigen::Vector4d& material) {
  drake::lcmt_viewer_geometry_data geometry_data;

  geometry_data.type = geometry_data.BOX;
  geometry_data.num_float_data = 3;
  geometry_data.float_data.push_back(static_cast<float>(box_size(0)));
  geometry_data.float_data.push_back(static_cast<float>(box_size(1)));
  geometry_data.float_data.push_back(static_cast<float>(box_size(2)));
  // Saves the location and orientation of the visualization geometry in the
  // `lcmt_viewer_geometry_data` object. The location and orientation are
  // specified in the body's frame.
  Eigen::Map<Eigen::Vector3f> position(geometry_data.position);
  position = local_transform.translation().cast<float>();
  Eigen::Map<Eigen::Vector4f> quaternion(geometry_data.quaternion);
  quaternion = drake::math::RotationMatrix<double>::ToQuaternionAsVector4(local_transform.linear()).cast<float>();

  Eigen::Map<Eigen::Vector4f> color(geometry_data.color);
  color = material.template cast<float>();

  return geometry_data;
}

}  // namespace

template <typename T>
BoxCarVis<T>::BoxCarVis(int model_instance_id, const std::string& name) : CarVis<T>(model_instance_id, name) {
  drake::lcmt_viewer_link_data link_data;
  link_data.name = CarVis<T>::name();
  link_data.robot_num = CarVis<T>::id();
  link_data.num_geom = 1;
  link_data.geom.resize(1);
  link_data.geom[0] = MakeBoxGeometryData(Eigen::Vector3d(1.0, 1.0, 1.0) /* box size */,
                                          Eigen::Isometry3d::Identity() /* element to local transform */,
                                          Eigen::Vector4d(0.7, 0.7, 0.7, 1) /* material */);

  vis_elements_.push_back(link_data);
}

template <typename T>
const vector<drake::lcmt_viewer_link_data>& BoxCarVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
PoseBundle<T> BoxCarVis<T>::CalcPoses(const drake::Isometry3<T>& X_WM) const {
  PoseBundle<T> result(1);

  result.set_transform(0, RigidTransform<T>(X_WM));
  result.set_name(0, this->name());
  result.set_model_instance_id(0, this->id());
  return result;
}

// These instantiations must match the API documentation in
// box_car_vis.h.
template class BoxCarVis<double>;

}  // namespace delphyne
