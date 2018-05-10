// Copyright 2017 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>

#include <drake/automotive/car_vis_applicator.h>
#include <drake/lcm/drake_lcm_interface.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/PluginMacros.hh>

#include "./types.h"
#include "linb-any"

namespace delphyne {
/// The abstract class that all plugins must inherit from.  Concrete
/// implementations must implement both the 'Configure' method and the
/// 'Initialize' method; see the documentation for those methods for more
/// information.
///
/// @tparam T must generally be a "double-like" type.  Currently, the supported
///           template types are 'double', 'drake::AutoDiffXd', and
///           'drake::symbolic::Expression'.
template <typename T>
class AgentPluginBase {
public:
  virtual ~AgentPluginBase() {}

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
  /// that the automotive simulator is building.
  virtual int Configure(
      const std::string& name,
      const int& id,
      const std::map<std::string, linb::any>& parameters,
      drake::systems::DiagramBuilder<T>* builder,
      drake::systems::rendering::PoseAggregator<T>* aggregator,
      drake::automotive::CarVisApplicator<T>* car_vis_applicator) = 0;

  /// The Initialize method is called right before the simualtion starts.  This
  /// gives plugins a chance to initialize themselves for running.
  virtual int Initialize(drake::systems::Context<T>* context) = 0;

  const std::string& get_name() { return name_; }
  void set_name(const std::string& name) { name_ = name; }
  const int& get_id() { return id_; }
  void set_id(const int& id) { id_ = id; }
  void set_plugin(ignition::common::PluginPtr plugin) { plugin_ = plugin; }

  virtual drake::systems::System<T>* get_system() const = 0;

protected:
  // Store a pointer (actually a shared_ptr) to the plugin within this class.
  // this is needed so that the plugin pointer doesn't go out of scope and get
  // freed while it is still in use.
  ignition::common::PluginPtr plugin_;
  // This id should be set by the simulator who is in charge of ensuring each
  // agent (system) in the simulation receives a unique id that can be passed
  // to register inputs on the pose aggregator
  int id_;
  std::string name_;
};

typedef AgentPluginBase<double> AgentPlugin;
typedef AgentPluginBase<AutoDiff> AutoDiffAgentPlugin;
typedef AgentPluginBase<Symbolic> SymbolicAgentPlugin;

/// The abstract class factory that all plugins must inherit from.  Concrete
/// implementations must implement the 'Create' method; see the documentation
/// for that method for more information.
///
/// @tparam T must generally be a "double-like" type.  Currently, the supported
///           template types are 'double', 'drake::AutoDiffXd', and
///           'drake::symbolic::Expression'.
template <typename T>
class AgentPluginFactoryBase {
 public:
  /// The `Create` method is used to get a std::unique_ptr of the concrete
  /// class that inherited from from AgentPluginBase.
  virtual std::unique_ptr<AgentPluginBase<T>> Create() = 0;

  /// Default destructor
  virtual ~AgentPluginFactoryBase() = default;
};

typedef AgentPluginFactoryBase<double> AgentPluginFactory;
typedef AgentPluginFactoryBase<AutoDiff> AutoDiffAgentPluginFactory;
typedef AgentPluginFactoryBase<Symbolic> SymbolicAgentPluginFactory;

/// Traits lookup for the agent plugin factories.
template <typename T>
struct AgentPluginFactoryTraits {
  static const char* name() { return "unknown"; }
};

// a specialization for each type of those you want to support
// and don't like to rely on the implementation-defined string
// returned by typeid
template <>
struct AgentPluginFactoryTraits<double> {
  static const char* name() { return "::delphyne::AgentPluginFactory"; }
};

template <>
struct AgentPluginFactoryTraits<AutoDiff> {
  static const char* name() { return "::delphyne::AutoDiffAgentPluginFactory"; }
};

template <>
struct AgentPluginFactoryTraits<Symbolic> {
  static const char* name() { return "::delphyne::SymbolicAgentPluginFactory"; }
};

}  // namespace delphyne
