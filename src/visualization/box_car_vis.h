// Copyright 2018 Toyota Research Institute
#pragma once

#include <string>
#include <vector>

#include <drake/systems/rendering/pose_bundle.h>
#include <drake/lcmt_viewer_link_data.hpp>

#include "visualization/car_vis.h"

namespace delphyne {

/// BoxCarVis displays a box as the visual representation of a vehicle.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
template <typename T>
class BoxCarVis : public CarVis<T> {
 public:
  BoxCarVis(int model_instance_id, const std::string& name);

  const std::vector<drake::lcmt_viewer_link_data>& GetVisElements() const override;

  drake::systems::rendering::PoseBundle<T> CalcPoses(const drake::Isometry3<T>& X_WM) const override;

 private:
  std::vector<drake::lcmt_viewer_link_data> vis_elements_;
};

}  // namespace delphyne
