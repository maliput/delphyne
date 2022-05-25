// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
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

#include "agents/trajectory_agent.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <drake/common/eigen_types.h>
#include <maliput/common/maliput_unused.h>

#include "systems/trajectory.h"
#include "systems/trajectory_follower.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

// TODO(daniel.stonier) convert this to accepting a Trajectory class instead
TrajectoryAgentBlueprint::TrajectoryAgentBlueprint(const std::string& name, const std::vector<double>& times,
                                                   const std::vector<double>& headings,
                                                   const std::vector<std::vector<double>>& translations)
    : BasicAgentBlueprint(name) {
  std::vector<Eigen::Quaternion<double>> eigen_orientations;
  for (const double& heading : headings) {
    Eigen::Quaternion<double> orientation(Eigen::AngleAxis<double>(heading, Eigen::Vector3d::UnitZ()));
    eigen_orientations.push_back(orientation);
  }
  std::vector<Eigen::Vector3d> eigen_translations;
  for (const std::vector<double>& translation : translations) {
    // TODO(daniel.stonier) assert on size 3, but we'll instead be switching to
    // accepting trajectories here, do it later
    Eigen::Vector3d eigen_translation;
    eigen_translation << translation[0], translation[1], translation[2];
    eigen_translations.push_back(eigen_translation);
  }

  trajectory_ = std::make_unique<Trajectory>(Trajectory::Make(times, eigen_orientations, eigen_translations));

  // TODO(daniel.stonier) stop using this, make use of an initial value on
  // the pose output
  trajectory_->value(times.front());
}

std::unique_ptr<Agent::Diagram> TrajectoryAgentBlueprint::DoBuildDiagram(
    maliput::api::RoadNetwork* road_network) const {
  maliput::common::unused(road_network);
  AgentBlueprint::DiagramBuilder builder(this->name());

  /******************************************
   * Trajectory Follower System
   ******************************************/
  // TODO(daniel.stonier) have this sample on update events from the simulation
  // than arbitrarily choosing it's own update rate.
  double sampling_time = 0.01;

  typedef TrajectoryFollower<double> TrajectoryFollower;
  TrajectoryFollower* trajectory_follower_system =
      builder.AddSystem(std::make_unique<TrajectoryFollower>(*trajectory_, sampling_time));
  trajectory_follower_system->set_name(this->name() + "_system");

  /*********************
   * Diagram Outputs
   *********************/
  builder.ExportStateOutput(trajectory_follower_system->state_output());
  builder.ExportPoseOutput(trajectory_follower_system->pose_output());
  builder.ExportVelocityOutput(trajectory_follower_system->velocity_output());

  return builder.Build();
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
