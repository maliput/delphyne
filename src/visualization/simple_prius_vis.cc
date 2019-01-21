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
#ifdef HAVE_SPDLOG
  // Avoid the many & varied 'warn' level logging messages coming from drake.
  //
  // Note: Drake will have defined HAVE_SPDLOG if it is using that
  // (see lib/cmake/spdlog/spdlog-config.cmake that was installed by drake).
  drake::log()->set_level(spdlog::level::err);
#endif
  const char* delphyne_resource_root = std::getenv("DELPHYNE_RESOURCE_ROOT");
  DELPHYNE_DEMAND(delphyne_resource_root != NULL);
  std::stringstream sdf_filename;
  sdf_filename << delphyne_resource_root << "/media/prius/simple_prius.sdf";
  plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  drake::multibody::Parser parser(&plant_);
  prius_index_ = parser.AddModelFromFile(sdf_filename.str());
  plant_.Finalize();

  plant_context_ = plant_.CreateDefaultContext();

  drake::lcm::DrakeMockLcm lcm;
  DispatchLoadMessage(scene_graph_, &lcm);
  auto load_message = lcm.DecodeLastPublishedMessageAs<
    drake::lcmt_viewer_load_robot>("DRAKE_VIEWER_LOAD_ROBOT");
  for (auto& link : load_message.link) {
    link.name = link.name.substr(link.name.find("::") + 2);
    vis_elements_.push_back(link);
    vis_elements_.back().robot_num = id;
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
  const drake::multibody::Body<T>& footprint =
      plant_.GetBodyByName("chassis_footprint");
  plant_.SetFreeBodyPose(plant_context_.get(), footprint, X_WM);

  int bundle_index = 0;
  std::vector<drake::multibody::BodyIndex> parts_indices =
      plant_.GetBodyIndices(prius_index_);
  drake::systems::rendering::PoseBundle<T> result(vis_elements_.size());
  for (drake::lcmt_viewer_link_data link_data : vis_elements_) {
    const drake::multibody::Body<T>& part =
        plant_.GetBodyByName(link_data.name);
    const drake::Isometry3<T>& X_WP =
        plant_.EvalBodyPoseInWorld(*plant_context_, part);
    result.set_pose(bundle_index, X_WP);
    result.set_name(bundle_index, part.name());
    result.set_model_instance_id(bundle_index, this->id());
    ++bundle_index;
  }
  return result;
}

// These instantiations must match the API documentation in
// prius_vis.h.
template class SimplePriusVis<double>;

}  // namespace delphyne
