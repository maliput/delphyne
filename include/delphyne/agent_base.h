// Copyright 2017 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>

#include <drake/automotive/car_vis_applicator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "./types.h"

namespace delphyne {
/// The abstract class that all delphyne agents must inherit from. Concrete
/// implementations must implement both the 'Configure' method and the
/// 'Initialize' method; see the documentation for those methods for more
/// information.
///
/// @tparam T must generally be a "double-like" type.  Currently, the supported
///           template types are 'double', 'drake::AutoDiffXd', and
///           'drake::symbolic::Expression'.
template <typename T>
class AgentBase {
 public:
  AgentBase(const std::string& name) : id_(0), name_(name) {}
  virtual ~AgentBase() = default;

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
      const int& id, drake::systems::DiagramBuilder<T>* builder,
      drake::systems::rendering::PoseAggregator<T>* aggregator,
      drake::automotive::CarVisApplicator<T>* car_vis_applicator) = 0;

  /// The Initialize method is called right before the simualtion starts.
  virtual int Initialize(drake::systems::Context<T>* context) = 0;

  const std::string& name() { return name_; }
  const int& id() { return id_; }

  virtual drake::systems::System<T>* get_system() const = 0;

 protected:
  // This id should be set by the simulator who is in charge of ensuring each
  // agent (system) in the simulation receives a unique id that can be passed
  // to register inputs on the pose aggregator
  int id_;
  std::string name_;
};

typedef AgentBase<double> Agent;
typedef AgentBase<AutoDiff> AutoDiffAgent;
typedef AgentBase<Symbolic> SymbolicAgent;

}  // namespace delphyne
