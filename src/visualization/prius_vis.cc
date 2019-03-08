// Copyright 2018-2019 Toyota Research Institute

#include "visualization/prius_vis.h"

#include <algorithm>
#include <iterator>
#include <list>
#include <string>
#include <vector>

#include <drake/common/drake_assert.h>
#include <drake/common/eigen_types.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_mock_lcm.h>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>

#include <ignition/common/SystemPaths.hh>

#include "delphyne/macros.h"

#include "backend/geometry_utilities.h"

namespace delphyne {

template <typename T>
constexpr double PriusVis<T>::kVisOffset;

template <typename T>
PriusVis<T>::PriusVis(int id, const std::string& name)
    : CarVis<T>(id, name) {
#ifdef HAVE_SPDLOG
  // Avoid the many & varied 'warn' level logging messages coming from drake.
  //
  // Note: Drake will have defined HAVE_SPDLOG if it is using that
  // (see lib/cmake/spdlog/spdlog-config.cmake that was installed by drake).
  drake::log()->set_level(spdlog::level::err);
#endif
  using ignition::common::SystemPaths;
  const std::list<std::string> paths =
      SystemPaths::PathsFromEnv("DELPHYNE_RESOURCE_ROOT");
  DELPHYNE_VALIDATE(!paths.empty(), std::runtime_error,
                    "DELPHYNE_RESOURCE_ROOT environment "
                    "variable is not set");
  std::vector<std::string> resource_paths;
  resource_paths.reserve(paths.size());
  std::copy(paths.begin(), paths.end(), std::back_inserter(resource_paths));
  const std::string sdf_path = SystemPaths::LocateLocalFile(
      "media/prius/prius_with_lidar.sdf", resource_paths);
  plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  drake::multibody::Parser parser(&plant_);
  prius_index_ = parser.AddModelFromFile(sdf_path);
  plant_.Finalize();

  plant_context_ = plant_.CreateDefaultContext();

  drake::lcmt_viewer_load_robot load_message = BuildLoadMessage(scene_graph_);
  for (drake::lcmt_viewer_link_data& link : load_message.link) {
    link.name = link.name.substr(link.name.rfind("::") + 2);
    link.robot_num = id;
    vis_elements_.push_back(link);
  }
}

template <typename T>
const std::vector<drake::lcmt_viewer_link_data>&
PriusVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
drake::systems::rendering::PoseBundle<T>
PriusVis<T>::CalcPoses(const drake::Isometry3<T>& X_WM) const {
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
template class PriusVis<double>;

}  // namespace delphyne
