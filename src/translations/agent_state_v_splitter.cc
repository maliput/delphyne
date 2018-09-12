// Copyright 2018 Toyota Research Institute

#include "translations/agent_state_v_splitter.h"

#include <functional>
#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"

#include "delphyne/macros.h"
#include "delphyne/protobuf/agent_state.pb.h"
#include "delphyne/protobuf/agent_state_v.pb.h"

namespace delphyne {

template <typename T>
AgentState_v_Splitter<T>::AgentState_v_Splitter(int num_agents) {
  DELPHYNE_VALIDATE(num_agents > 0, std::invalid_argument,
                    "There must be at least 1 agent.");
  this->DeclareAbstractInputPort();

  auto do_alloc = std::bind(&AgentState_v_Splitter<T>::DoAlloc, this);
  using std::placeholders::_1;
  using std::placeholders::_2;
  for (int i = 0; i < num_agents; ++i) {
    auto do_split =
        std::bind(&AgentState_v_Splitter<T>::DoSplit, this, _1, _2, i);
    this->DeclareAbstractOutputPort(do_alloc, do_split);
  }
}

template <typename T>
std::unique_ptr<drake::systems::AbstractValue>
AgentState_v_Splitter<T>::DoAlloc() const {
  return drake::systems::AbstractValue::Make(state_);
}

template <typename T>
void AgentState_v_Splitter<T>::DoSplit(
    const drake::systems::Context<T>& context,
    drake::systems::AbstractValue* output, int agent_index) const {
  // Evaluates input.
  const ignition::msgs::AgentState_V& simple_car_state_v =
      this->EvalAbstractInput(context, 0)
          ->template GetValue<ignition::msgs::AgentState_V>();

  // Assigns the state returned by the agent_index to the output.
  auto& mutable_state = output->GetMutableValue<ignition::msgs::AgentState>();
  if (simple_car_state_v.states_size() > agent_index) {
    mutable_state = simple_car_state_v.states(agent_index);
  }
}

template class AgentState_v_Splitter<double>;

}  // namespace delphyne
