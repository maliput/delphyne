// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <map>
#include <memory>
#include <string>

#include "backend/linb-any"
#include "backend/system.h"

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/PluginMacros.hh>

#include "drake/automotive/car_vis_applicator.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_aggregator.h"

namespace delphyne {
namespace backend {
/// The abstract class that all plugins must inherit from.  Concrete
/// implementations must implement both the 'Configure' method and the
/// 'Initialize' method; see the documentation for those methods for more
/// information.
///
/// @tparam T must generally be a "double-like" type.  Currently, the supported
///           template types are 'double', 'drake::AutoDiffXd', and
///           'drake::symbolic::Expression'.
template <typename T>
class DELPHYNE_BACKEND_VISIBLE AgentPluginBase
    : public drake::systems::LeafSystem<T> {
 public:
  /// The `Configure` method is the main way that loadable agents get the
  /// information that they need to insert themselves into an automotive
  /// simulation.  Concrete implementations should set themselves up and insert
  /// themselves into the simulation during the `Configure` call.  The
  /// `parameters` argument is a map between string names and "linb::any", which
  /// is a drop-in replacement for std::any for older compilers.  This map is
  /// meant to hold agent-specific arguments that need to be passed down from
  /// the application into the loadable agent for it to properly configure
  /// itself.  The rest of the arguments are parameters that are needed by all
  /// (or at least most) loadable agents to insert themselves into the
  /// simulation.  For instance, the `builder` parameter is typically used by
  /// the loadable agent to connect internal methods into the overall Diagram
  /// that the automotive simulator is building.  The `lcm` parameter is used
  /// to attach an LCM subscriber or publisher to the concrete agent.
  virtual int Configure(
      const std::map<std::string, linb::any>& parameters,
      drake::systems::DiagramBuilder<T>* builder,
      drake::lcm::DrakeLcmInterface* lcm, const std::string& name, int id,
      drake::systems::rendering::PoseAggregator<T>* aggregator,
      drake::automotive::CarVisApplicator<T>* car_vis_applicator) = 0;

  /// The Initialize method is called right before the simualtion starts.  This
  /// gives plugins a chance to initialize themselves for running.
  virtual int Initialize(drake::systems::Context<T>* context) = 0;

  void SetPlugin(ignition::common::PluginPtr plugin) { plugin_ = plugin; }

 protected:
  // Store a pointer (actually a shared_ptr) to the plugin within this class.
  // this is needed so that the plugin pointer doesn't go out of scope and get
  // freed while it is still in use.
  ignition::common::PluginPtr plugin_;
};

typedef delphyne::backend::AgentPluginBase<double> AgentPluginDoubleBase;
typedef delphyne::backend::AgentPluginBase<::drake::AutoDiffXd>
    AgentPluginAutoDiffXdBase;
typedef delphyne::backend::AgentPluginBase<::drake::symbolic::Expression>
    AgentPluginExpressionBase;

/// The abstract class factory that all plugins must inherit from.  Concrete
/// implementations must implement the 'Create' method; see the documentation
/// for that method for more information.
///
/// @tparam T must generally be a "double-like" type.  Currently, the supported
///           template types are 'double', 'drake::AutoDiffXd', and
///           'drake::symbolic::Expression'.
template <typename T>
class DELPHYNE_BACKEND_VISIBLE AgentPluginFactoryBase {
 public:
  /// The `Create` method is used to get a std::unique_ptr of the concrete
  /// class that inherited from from AgentPluginBase.
  virtual std::unique_ptr<AgentPluginBase<T>> Create() = 0;
};

typedef delphyne::backend::AgentPluginFactoryBase<double>
    AgentPluginFactoryDoubleBase;
typedef delphyne::backend::AgentPluginFactoryBase<::drake::AutoDiffXd>
    AgentPluginFactoryAutoDiffXdBase;
typedef delphyne::backend::AgentPluginFactoryBase<::drake::symbolic::Expression>
    AgentPluginFactoryExpressionBase;

}  // namespace backend
}  // namespace delphyne