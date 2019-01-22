/**
 * @file src/visualization/simple_prius_vis.h
 *
 * Copyright 2018-2019 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <drake/common/drake_copyable.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/rendering/pose_bundle.h>

#include "visualization/car_vis.h"

namespace delphyne {

/**
 * Note: this class is based on (aka copy-of) the PriusVis class in Drake
 * https://github.com/RobotLocomotion/drake/blob/master/automotive/prius_vis.h
 *
 * SimplePriusVis displays a visualization of a 2015 Toyota Prius. It relies on
 * `media/prius/simple_prius.sdf` and requires that this SDF file only contain
 * one model instance that is not connected to the world.
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

  const std::vector<drake::lcmt_viewer_link_data>& GetVisElements()
      const override;

  drake::systems::rendering::PoseBundle<T> CalcPoses(
      const drake::Isometry3<T>& X_WM) const override;

 private:
  drake::geometry::SceneGraph<T> scene_graph_{};
  drake::multibody::MultibodyPlant<T> plant_{};
  drake::multibody::ModelInstanceIndex prius_index_{};
  std::unique_ptr<drake::systems::Context<T>> plant_context_{nullptr};
  std::vector<drake::lcmt_viewer_link_data> vis_elements_{};
};

}  // namespace delphyne
