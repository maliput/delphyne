// Copyright 2022 Toyota Research Institute
#include "backend/ign_commanded_pass_through.h"

#include <ignition/msgs.hh>

namespace delphyne {

template <typename T>
IgnCommandedPassThrough<T>::IgnCommandedPassThrough() {
  data_input_port_index_ = DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<T>()).get_index();

  switch_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<ignition::msgs::Boolean>()).get_index();

  data_output_port_index_ = DeclareAbstractOutputPort(&IgnCommandedPassThrough::CalcOutput).get_index();
}

template <typename T>
void IgnCommandedPassThrough<T>::CalcOutput(const drake::systems::Context<double>& context, T* output_ids) const {
  // Retrieves ids and states inputs.
  const T* input_ids = this->template EvalInputValue<T>(context, data_input_port_index_);

  const ignition::msgs::Boolean* input_switch =
      this->template EvalInputValue<ignition::msgs::Boolean>(context, switch_input_port_index_);

  output_ids->Clear();

  if (input_switch->data()) {
    *output_ids = *input_ids;
  }
}

template class IgnCommandedPassThrough<ignition::msgs::UInt32_V>;

}  // namespace delphyne
