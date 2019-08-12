// Copyright 2018-2019 Toyota Research Institute

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

/// PriusVis displays a visualization of a 2015 Toyota Prius. It relies on
/// `media/prius/prius_with_lidar.sdf` and requires that this
/// SDF file only contain one model instance that is not connected to the world.
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

  const std::vector<drake::lcmt_viewer_link_data>& GetVisElements() const override;

  drake::systems::rendering::PoseBundle<T> CalcPoses(const drake::Isometry3<T>& X_WM) const override;

 private:
  drake::geometry::SceneGraph<T> scene_graph_{};
  drake::multibody::MultibodyPlant<T> plant_{};
  drake::multibody::ModelInstanceIndex prius_index_{};
  std::unique_ptr<drake::systems::Context<T>> plant_context_{nullptr};
  std::vector<drake::lcmt_viewer_link_data> vis_elements_{};
};

}  // namespace delphyne
