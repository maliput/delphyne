// Copyright 2018 Toyota Research Institute

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <drake/common/drake_copyable.h>
#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/rendering/pose_bundle.h>

#include "visualization/car_vis.h"

namespace delphyne {

/// PriusVis displays a visualization of a 2015 Toyota Prius. It relies on
/// `media/prius/prius_with_lidar.sdf` and requires that this
/// SDF file only contain one model instance that is not connected to the world
/// (i.e., the root body of the SDF model must not be named
/// RigidBodyTreeConstants::kWorldName).
///
/// Note also that this class is included in Delphyne for completeness sake.
/// Currently the car simulations are using SimplePriusVis.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
template <typename T>
class PriusVis : public CarVis<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PriusVis)

  /// Defines the distance between the visual model's origin and the middle of
  /// the rear axle.
  static constexpr double kVisOffset{1.40948};

  PriusVis(int id, const std::string& name);

  const std::vector<drake::lcmt_viewer_link_data>& GetVisElements()
      const override;

  drake::systems::rendering::PoseBundle<T> CalcPoses(
      const drake::Isometry3<T>& X_WM) const override;

 private:
  std::unique_ptr<RigidBodyTree<T>> tree_;
  std::vector<drake::lcmt_viewer_link_data> vis_elements_;
};

}  // namespace delphyne
