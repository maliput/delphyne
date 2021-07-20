// Copyright 2017 Toyota Research Institute

#include "delphyne/mi6/agent_simulation.h"

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <drake/common/eigen_types.h>
#include <drake/common/text_logging.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/systems/analysis/runge_kutta2_integrator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/framework/system_output.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>

#include "backend/scene_system.h"
#include "delphyne/macros.h"
#include "delphyne/protobuf/agent_state.pb.h"
#include "delphyne/protobuf/agent_state_v.pb.h"

namespace delphyne {

template <typename T>
AgentSimulationBase<T>::AgentSimulationBase(std::unique_ptr<drake::systems::Simulator<T>> simulator,
                                            std::unique_ptr<drake::systems::Diagram<T>> diagram,
                                            std::map<std::string, std::unique_ptr<AgentBase<T>>> agents,
                                            std::unique_ptr<const maliput::api::RoadGeometry> road_geometry,
                                            drake::geometry::SceneGraph<T>* scene_graph, SceneSystem* scene_system)
    : simulator_(std::move(simulator)),
      diagram_(std::move(diagram)),
      agents_(std::move(agents)),
      road_geometry_(std::move(road_geometry)),
      road_network_(nullptr),
      scene_graph_(scene_graph),
      scene_system_(scene_system) {
  DELPHYNE_VALIDATE(simulator_ != nullptr, std::runtime_error, "Invalid null simulator given");
  DELPHYNE_VALIDATE(diagram_ != nullptr, std::runtime_error, "Invalid null diagram given");
  DELPHYNE_VALIDATE(scene_graph_ != nullptr, std::runtime_error, "Invalid null scene graph given");
  DELPHYNE_VALIDATE(scene_graph_ != nullptr, std::runtime_error, "Invalid null scene graph given");
  DELPHYNE_VALIDATE(scene_system_ != nullptr, std::runtime_error, "Invalid null scene system given");
  // Retrieve SceneGraph query object for later use.
  const drake::systems::OutputPort<T>& scene_graph_query_port = scene_graph_->get_query_output_port();
  scene_query_ = scene_graph_query_port.Allocate();
  const drake::systems::Context<T>& scene_graph_context =
      diagram_->GetSubsystemContext(*scene_graph_, simulator_->get_context());
  scene_graph_query_port.Calc(scene_graph_context, scene_query_.get());
}

template <typename T>
AgentSimulationBase<T>::AgentSimulationBase(std::unique_ptr<drake::systems::Simulator<T>> simulator,
                                            std::unique_ptr<drake::systems::Diagram<T>> diagram,
                                            std::map<std::string, std::unique_ptr<AgentBase<T>>> agents,
                                            std::unique_ptr<const maliput::api::RoadNetwork> road_network,
                                            drake::geometry::SceneGraph<T>* scene_graph, SceneSystem* scene_system)
    : simulator_(std::move(simulator)),
      diagram_(std::move(diagram)),
      agents_(std::move(agents)),
      road_geometry_(nullptr),
      road_network_(std::move(road_network)),
      scene_graph_(scene_graph),
      scene_system_(scene_system) {
  DELPHYNE_VALIDATE(simulator_ != nullptr, std::runtime_error, "Invalid null simulator given");
  DELPHYNE_VALIDATE(diagram_ != nullptr, std::runtime_error, "Invalid null diagram given");
  DELPHYNE_VALIDATE(scene_graph_ != nullptr, std::runtime_error, "Invalid null scene graph given");
  DELPHYNE_VALIDATE(scene_graph_ != nullptr, std::runtime_error, "Invalid null scene graph given");
  DELPHYNE_VALIDATE(scene_system_ != nullptr, std::runtime_error, "Invalid null scene system given");
  // Retrieve SceneGraph query object for later use.
  const drake::systems::OutputPort<T>& scene_graph_query_port = scene_graph_->get_query_output_port();
  scene_query_ = scene_graph_query_port.Allocate();
  const drake::systems::Context<T>& scene_graph_context =
      diagram_->GetSubsystemContext(*scene_graph_, simulator_->get_context());
  scene_graph_query_port.Calc(scene_graph_context, scene_query_.get());
}

template <typename T>
std::unique_ptr<ignition::msgs::Scene> AgentSimulationBase<T>::GetVisualScene() {
  auto scene_message = std::make_unique<ignition::msgs::Scene>();

  // The scene is the output of the scene system. We could use
  // LeafOutputPort::Eval to directly retrieve the last calculated value, but
  // that function is as of now unimplemented, so we need to recalculate it.
  // Since this doesn't happen often (once per scene request), this is not an
  // issue.
  const drake::systems::Context<T>& scene_context =
      diagram_->GetSubsystemContext(*scene_system_, simulator_->get_context());

  using drake::systems::SystemOutput;
  std::unique_ptr<SystemOutput<T>> output = scene_system_->AllocateOutput();
  scene_system_->CalcOutput(scene_context, output.get());

  scene_message->CopyFrom(output->get_data(0)->template get_value<ignition::msgs::Scene>());

  return scene_message;
}

template <typename T>
const delphyne::AgentBase<T>& AgentSimulationBase<T>::GetAgentByName(const std::string& name) const {
  DELPHYNE_VALIDATE(agents_.count(name) == 1, std::runtime_error, "No agent found with the given name.");
  return *agents_.at(name);
}

template <typename T>
delphyne::AgentBase<T>* AgentSimulationBase<T>::GetMutableAgentByName(const std::string& name) {
  DELPHYNE_VALIDATE(agents_.count(name) == 1, std::runtime_error, "No agent found with the given name.");
  return agents_[name].get();
}

namespace {

// A helper functor to check for a given object source (e.g. a GeometryId)
// on agent ID to agent mappings. Useful in combination with the STL algorithm
// library.
//
// @tparam T Agents' scalar type.
// @tparam U Object type.
// @see delphyne::AgentBase::IsSourceOf()
template <typename T, typename U>
struct IsSourceOf {
  // Constructs the functor with given @p object_in reference.
  // @remarks As only a reference to the object is kept (to keep it
  //          lightweight), the caller MUST ensure that the object
  //          instance outlives the functor instance.
  explicit IsSourceOf(const U& object_in) : object(object_in) {}

  // Checks whether the given (agent ID, agent) pair is the source of the
  // associated `object`.
  bool operator()(const std::pair<const std::string, std::unique_ptr<AgentBase<T>>>& name_agent) {
    return name_agent.second->IsSourceOf(object);
  }

  // Associated object to check the source of.
  const U& object;
};

}  // namespace

template <typename T>
std::vector<AgentBaseCollision<T>> AgentSimulationBase<T>::GetCollisions() const {
  using drake::geometry::GeometryId;
  using drake::geometry::PenetrationAsPointPair;
  using drake::geometry::QueryObject;
  const std::vector<PenetrationAsPointPair<T>> collisions =
      scene_query_->get_value<QueryObject<T>>().ComputePointPairPenetration();
  std::vector<AgentBaseCollision<T>> agent_collisions;
  for (const auto& collision : collisions) {
    const auto it_A = std::find_if(agents_.begin(), agents_.end(), IsSourceOf<T, GeometryId>(collision.id_A));
    DELPHYNE_VALIDATE(it_A != agents_.end(), std::runtime_error, "Could not find first agent in list of agents");
    const auto it_B = std::find_if(agents_.begin(), agents_.end(), IsSourceOf<T, GeometryId>(collision.id_B));
    DELPHYNE_VALIDATE(it_B != agents_.end(), std::runtime_error, "Could not find second agent in list of agents");
    agent_collisions.emplace_back(std::make_pair(it_A->second.get(), it_B->second.get()), collision.p_WCa);
  }
  return agent_collisions;
}

template <typename T>
void AgentSimulationBase<T>::StepBy(const T& time_step) {
  simulator_->AdvanceTo(GetCurrentTime() + time_step);
}

template <typename T>
drake::systems::rendering::PoseBundle<T> AgentSimulationBase<T>::GetCurrentPoses() const {
  using drake::AbstractValue;
  using drake::systems::SystemOutput;
  using drake::systems::rendering::PoseBundle;
  std::unique_ptr<SystemOutput<T>> system_output = diagram_->AllocateOutput();
  diagram_->CalcOutput(simulator_->get_context(), system_output.get());
  const AbstractValue* abstract_value = system_output->get_data(0);
  return abstract_value->get_value<PoseBundle<T>>();
}

template class AgentSimulationBase<double>;

}  // namespace delphyne
