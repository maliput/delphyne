// Copyright 2017 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>

#include <drake/automotive/car_vis_applicator.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginLoader.hh>
#include <ignition/common/PluginMacros.hh>

#include "./types.h"
#include "backend/system.h"

namespace delphyne {

/// The abstract class that models the extra required parameters for a plugin
/// to be configures. Plugin concrete classes can create a subclass of this
/// class and downcast to it on the `Configure` method (see the
/// `downcast_params` protected method). Admittedly, Barbara Liskov wouldn't be
/// happy about this approach.
class AgentPluginParams {
 public:
  virtual ~AgentPluginParams() {}
};

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
  /// `parameters` argument is a class that is meant to hold agent-specific
  /// arguments that need to be passed down from the application into the
  /// loadable agent for it to properly configure itself. Each concrete agent
  /// can create a specific subclass to explicitly state the parameters it
  /// needs. The rest of the arguments are parameters that are needed by all
  /// (or at least most) loadable agents to insert themselves into the
  /// simulation.  For instance, the `builder` parameter is typically used by
  /// the loadable agent to connect internal methods into the overall Diagram
  /// that the automotive simulator is building.
  virtual int Configure(
      const std::string& name, int id,
      drake::systems::DiagramBuilder<T>* builder,
      drake::systems::rendering::PoseAggregator<T>* aggregator,
      drake::automotive::CarVisApplicator<T>* car_vis_applicator,
      const drake::maliput::api::RoadGeometry* road,
      std::unique_ptr<AgentPluginParams> parameters) = 0;

  /// The Initialize method is called right before the simualtion starts.  This
  /// gives plugins a chance to initialize themselves for running.
  virtual int Initialize(drake::systems::Context<T>* context) = 0;

  const std::string& get_name() { return name_; }
  const int& get_id() { return id_; }
  void set_plugin(ignition::common::PluginPtr plugin) { plugin_ = plugin; }

  virtual drake::systems::System<T>* get_system() const = 0;

 protected:
  /// Performs a parameter downcast to the concrete agent params class.
  /// Throws an exception if the provided parameter is not of the expected
  /// class.
  template <typename ConcreteAgentPluginParams,
            typename std::enable_if<std::is_base_of<
                AgentPluginParams, ConcreteAgentPluginParams>::value>::type* =
                nullptr>
  std::unique_ptr<ConcreteAgentPluginParams> downcast_params(
      std::unique_ptr<AgentPluginParams> base_params) {
    DELPHYNE_DEMAND(base_params.get() != nullptr);

    AgentPluginParams* const raw_base_params = base_params.release();

    ConcreteAgentPluginParams* const concrete_parameters =
        dynamic_cast<ConcreteAgentPluginParams*>(raw_base_params);

    if (concrete_parameters == nullptr) {
      throw std::runtime_error(
          "Provided parameters can't be downcasted to the expected type");
    }

    return std::unique_ptr<ConcreteAgentPluginParams>(concrete_parameters);
  }

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
