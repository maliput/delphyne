// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*****************************************************************************
** Includes
*****************************************************************************/

#include "visualization/simple_prius_vis.h"

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
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>
#include <ignition/common/SystemPaths.hh>

#include "backend/geometry_utilities.h"
#include "delphyne/macros.h"

namespace delphyne {

using drake::math::RigidTransform;

template <typename T>
constexpr double SimplePriusVis<T>::kVisOffset;

template <typename T>
SimplePriusVis<T>::SimplePriusVis(int id, const std::string& name) : CarVis<T>(id, name) {
#ifdef HAVE_SPDLOG
  // Avoid the many & varied 'warn' level logging messages coming from drake.
  //
  // Note: Drake will have defined HAVE_SPDLOG if it is using that
  // (see lib/cmake/spdlog/spdlog-config.cmake that was installed by drake).
  drake::log()->set_level(spdlog::level::err);
#endif
  using ignition::common::SystemPaths;
  const std::list<std::string> paths = SystemPaths::PathsFromEnv("DELPHYNE_RESOURCE_ROOT");
  DELPHYNE_VALIDATE(!paths.empty(), std::runtime_error,
                    "DELPHYNE_RESOURCE_ROOT environment "
                    "variable is not set");
  std::vector<std::string> resource_paths;
  resource_paths.reserve(paths.size());
  std::copy(paths.begin(), paths.end(), std::back_inserter(resource_paths));
  const std::string sdf_path = SystemPaths::LocateLocalFile("media/prius/simple_prius.sdf", resource_paths);
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
const std::vector<drake::lcmt_viewer_link_data>& SimplePriusVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
drake::systems::rendering::PoseBundle<T> SimplePriusVis<T>::CalcPoses(const drake::Isometry3<T>& X_WM) const {
  const drake::multibody::Body<T>& footprint = plant_.GetBodyByName("chassis_footprint");
  plant_.SetFreeBodyPose(plant_context_.get(), footprint, RigidTransform<T>(X_WM));

  int bundle_index = 0;
  std::vector<drake::multibody::BodyIndex> parts_indices = plant_.GetBodyIndices(prius_index_);
  drake::systems::rendering::PoseBundle<T> result(vis_elements_.size());
  for (const drake::lcmt_viewer_link_data& link_data : vis_elements_) {
    const drake::multibody::Body<T>& part = plant_.GetBodyByName(link_data.name);
    const drake::Isometry3<T> X_WP = plant_.EvalBodyPoseInWorld(*plant_context_, part).GetAsIsometry3();
    result.set_transform(bundle_index, RigidTransform<T>(X_WP));
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
