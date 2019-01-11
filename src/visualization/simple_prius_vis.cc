/**
 * @file src/visualization/simple_prius_vis.cc
 *
 * Copyright 2018-2019 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "visualization/simple_prius_vis.h"

#include <sstream>

#include <drake/common/drake_assert.h>
#include <drake/common/eigen_types.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_mock_lcm.h>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>

#include "delphyne/macros.h"

namespace delphyne {

template <typename T>
constexpr double SimplePriusVis<T>::kVisOffset;

template <typename T>
SimplePriusVis<T>::SimplePriusVis(int id, const std::string& name)
    : CarVis<T>(id, name) {
  const char* delphyne_resource_root = std::getenv("DELPHYNE_RESOURCE_ROOT");
  DELPHYNE_DEMAND(delphyne_resource_root != NULL);
  std::stringstream sdf_filename;
  sdf_filename << delphyne_resource_root << "/media/prius/simple_prius.sdf";
  plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  drake::multibody::Parser parser(&plant_);
  drake::multibody::ModelInstanceIndex prius_index =
      parser.AddModelFromFile(sdf_filename.str());
  plant_.Finalize();

  prius_parts_indices_ = plant_.GetBodyIndices(prius_index);
  plant_context_ = plant_.CreateDefaultContext();

  drake::lcm::DrakeMockLcm lcm;
  DispatchLoadMessage(scene_graph_, &lcm);
  auto load_message = lcm.DecodeLastPublishedMessageAs<
    drake::lcmt_viewer_load_robot>("DRAKE_VIEWER_LOAD_ROBOT");
  for (const auto& link : load_message.link) {
    if (link.name != plant_.world_body().name()) {
      vis_elements_.push_back(link);
      vis_elements_.back().robot_num = id;
    }
  }
}

template <typename T>
const std::vector<drake::lcmt_viewer_link_data>&
SimplePriusVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
drake::systems::rendering::PoseBundle<T>
SimplePriusVis<T>::CalcPoses(const drake::Isometry3<T>& X_WM) const {
  // Computes X_MV, the transform from the visualization's frame to the model's
  // frame. The 'V' in the variable name stands for "visualization". This is
  // necessary because the model frame's origin is centered at the midpoint of
  // the vehicle's rear axle whereas the visualization frame's origin is
  // centered in the middle of a body called "chassis_floor". The axes of the
  // two frames are parallel with each other. However, the distance between the
  // origins of the two frames is 1.40948 m along the model's x-axis.
  const drake::Isometry3<T> X_MV(drake::Translation3<T>(
      T(1.40948) /* x offset */, T(0) /* y offset */, T(0) /* z offset */));
  const drake::Isometry3<T> X_WV = X_WM * X_MV;
  const drake::multibody::Body<T>& base_part =
      plant_.get_body(prius_parts_indices_[0]);
  plant_.SetFreeBodyPose(plant_context_.get(), base_part, X_WV);

  int result_index{0};
  drake::systems::rendering::PoseBundle<T> result(prius_parts_indices_.size());
  for (const drake::multibody::BodyIndex& part_index : prius_parts_indices_) {
    const drake::multibody::Body<T>& part = plant_.get_body(part_index);
    const drake::Isometry3<T>& X_WB =
        plant_.EvalBodyPoseInWorld(*plant_context_, part);
    result.set_pose(result_index, X_WB);
    result.set_name(result_index, part.name());
    result.set_model_instance_id(result_index, this->id());
    ++result_index;
  }
  return result;
}

// These instantiations must match the API documentation in
// prius_vis.h.
template class SimplePriusVis<double>;

}  // namespace delphyne
