/**
 * @file src/visualization/simple_prius_vis.h
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <drake/automotive/car_vis.h>
#include <drake/common/drake_copyable.h>
#include <drake/common/eigen_types.h>
#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/rendering/pose_bundle.h>

namespace delphyne {

using drake::automotive::CarVis;
using drake::systems::rendering::PoseBundle;
using drake::lcmt_viewer_link_data;
using drake::Isometry3;

/**
 * Note: this class is based on (aka copy-of) the PriusVis class in Drake
 * https://github.com/RobotLocomotion/drake/blob/master/automotive/prius_vis.h
 *
 * This is a temporary workaround to use a simpler SDF model; ideally we would
 * like to be able to parametrize the SDF used in PriusVis, but that is TBD.
 */

template <typename T>
class SimplePriusVis : public CarVis<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimplePriusVis)

  /// Defines the distance between the visual model's origin and the middle of
  /// the rear axle.
  static constexpr double kVisOffset{1.40948};

  SimplePriusVis(int id, const std::string& name);

  const std::vector<lcmt_viewer_link_data>& GetVisElements() const override;

  drake::systems::rendering::PoseBundle<T> CalcPoses(
      const Isometry3<T>& X_WM) const override;

 private:
  std::unique_ptr<RigidBodyTree<T>> tree_;
  std::vector<lcmt_viewer_link_data> vis_elements_;
};

}  // namespace delphyne
