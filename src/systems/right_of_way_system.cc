// Copyright 2022 Toyota Research Institute
#include "systems/right_of_way_system.h"

#include <drake/common/extract_double.h>
#include <drake/systems/rendering/pose_vector.h>
#include <maliput/api/regions.h>
#include <maliput/api/rules/discrete_value_rule_state_provider.h>
#include <maliput/api/rules/rule.h>
#include <maliput/base/rule_registry.h>

#include "systems/lane_direction.h"

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::Rule;

namespace delphyne {

template <typename T>
RightOfWaySystem<T>::RightOfWaySystem(maliput::api::RoadNetwork* road_network)
    : lane_state_input_port_index_(
          this->DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<LaneDirection>()).get_index()),
      pose_input_port_index_(this->DeclareVectorInputPort(drake::systems::rendering::PoseVector<T>()).get_index()),
      velocity_input_port_index_(this->DeclareVectorInputPort(drake::systems::BasicVector<T>(1)).get_index()),
      velocity_output_port_index_(this->DeclareVectorOutputPort(drake::systems::BasicVector<T>(1),
                                                                &RightOfWaySystem::CalcOutputVelocity,
                                                                {this->xc_ticket()})
                                      .get_index()),
      road_network_(road_network) {
  DELPHYNE_DEMAND(road_network_ != nullptr);
}

template <typename T>
const drake::systems::InputPort<T>& RightOfWaySystem<T>::lane_state_input() const {
  return this->get_input_port(lane_state_input_port_index_);
}

template <typename T>
const drake::systems::InputPort<T>& RightOfWaySystem<T>::velocity_input() const {
  return this->get_input_port(velocity_input_port_index_);
}

template <typename T>
const drake::systems::InputPort<T>& RightOfWaySystem<T>::pose_input() const {
  return this->get_input_port(pose_input_port_index_);
}

template <typename T>
const drake::systems::OutputPort<T>& RightOfWaySystem<T>::velocity_output() const {
  return this->get_output_port(velocity_output_port_index_);
}

template <typename T>
void RightOfWaySystem<T>::CalcOutputVelocity(const drake::systems::Context<T>& context,
                                             drake::systems::BasicVector<T>* output) const {
  // Obtains the inputs.
  const drake::systems::BasicVector<T>* vel_input =
      this->template EvalVectorInput<drake::systems::BasicVector>(context, velocity_input_port_index_);
  DELPHYNE_DEMAND(vel_input != nullptr);

  const drake::systems::rendering::PoseVector<T>* pose_input =
      this->template EvalVectorInput<drake::systems::rendering::PoseVector>(context, pose_input_port_index_);
  DELPHYNE_DEMAND(pose_input != nullptr);

  const LaneDirection* const lane_direction =
      this->template EvalInputValue<LaneDirection>(context, this->lane_state_input().get_index());
  DELPHYNE_DEMAND(lane_direction != nullptr);

  // Obtain the right of way rule state for lane and position.
  const maliput::api::InertialPosition inertial_pos(drake::ExtractDoubleOrThrow(pose_input->get_translation().x()),
                                                    drake::ExtractDoubleOrThrow(pose_input->get_translation().y()),
                                                    drake::ExtractDoubleOrThrow(pose_input->get_translation().z()));

  // TODO(https://github.com/ToyotaResearchInstitute/maliput/issues/476) Use DiscreteValueRuleStateProvider to get the
  // state given a position once PhaseBasedRightOfWayDiscreteValueRuleStateProvider is fixed. That method is way more
  // direct that following the intersection book path. Also, this path demands to have the phase ring defined in a
  // intersection book.
  const auto intersections = road_network_->intersection_book()->GetIntersections();
  auto intersection_it = std::find_if(intersections.begin(), intersections.end(),
                                      [&inertial_pos, rn = this->road_network_](const maliput::api::Intersection* i) {
                                        if (i->region().empty()) return false;
                                        return i->Includes(inertial_pos, rn->road_geometry());
                                      });

  // Set the default value to be returned, which is the input velocity being passed through.
  (*output)[0] = (*vel_input)[0];

  if (intersection_it == intersections.end() || !(*intersection_it)->DiscreteValueRuleStates().has_value()) {
    return;
  }

  const auto discrete_value_rule_states = (*intersection_it)->DiscreteValueRuleStates();
  // Find the Right-Of-Way rule that includes the inertial position of the agent's location.
  const auto discrete_value_rule_state_itr = std::find_if(
      (*discrete_value_rule_states).begin(), (*discrete_value_rule_states).end(),
      [&inertial_pos, rn = this->road_network_](const std::pair<Rule::Id, DiscreteValueRule::DiscreteValue>& id_value) {
        const auto rule = rn->rulebook()->GetDiscreteValueRule(id_value.first);
        return maliput::api::IsIncluded(inertial_pos, rule.zone().ranges(), rn->road_geometry());
      });

  if (discrete_value_rule_state_itr == (*discrete_value_rule_states).end()) {
    return;
  }

  if (discrete_value_rule_state_itr->second.value == "Stop") {
    // Typically, Right Of Way Rule types' state can be: Go, Stop and StopThenGo.
    // This system doesn't take into account the StopThenGo scenario.
    (*output)[0] = 0.;
    return;
  }
}

// template class RightOfWaySystem<double>;
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(class ::delphyne::RightOfWaySystem)

}  // namespace delphyne
