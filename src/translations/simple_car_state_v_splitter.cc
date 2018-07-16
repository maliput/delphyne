// Copyright 2018 Toyota Research Institute

#include "translations/simple_car_state_v_splitter.h"

#include <functional>
#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"

#include "delphyne/protobuf/simple_car_state.pb.h"
#include "delphyne/protobuf/simple_car_state_v.pb.h"

namespace delphyne {

using std::placeholders::_1;
using std::placeholders::_2;

SimpleCarState_v_Splitter::SimpleCarState_v_Splitter(int agents_number) {
  DeclareAbstractInputPort();

  for (int i = 0; i < agents_number; ++i) {
    auto f = std::bind(&SimpleCarState_v_Splitter::DoSplit, this, _1, _2, i);
    auto d = std::bind(&SimpleCarState_v_Splitter::DoAlloc, this);
    DeclareAbstractOutputPort(d, f);
  }
}

std::unique_ptr<drake::systems::AbstractValue>
SimpleCarState_v_Splitter::DoAlloc() {
  return drake::systems::AbstractValue::Make(ignition::msgs::SimpleCarState{});
}

void SimpleCarState_v_Splitter::DoSplit(
    const drake::systems::Context<double>& context,
    drake::systems::AbstractValue* output, int agent_index) const {
  // Evaluates input.
  const ignition::msgs::SimpleCarState_V& simple_car_state_v =
      EvalAbstractInput(context, 0)
          ->template GetValue<ignition::msgs::SimpleCarState_V>();

  // Assigns the state returned by the agent_index to the output.
  auto& valor = output->GetMutableValue<ignition::msgs::SimpleCarState>();
  if (simple_car_state_v.states_size() >= agent_index) {
    valor = simple_car_state_v.states(agent_index);
  }
}

}  // namespace delphyne
