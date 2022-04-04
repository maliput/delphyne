// Copyright 2022 Toyota Research Institute
#include "backend/ign_models_to_ids.h"

namespace delphyne {

IgnModelsToIds::IgnModelsToIds() {
  models_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<ignition::msgs::Model_V>()).get_index();

  ids_output_port_index_ = DeclareAbstractOutputPort(&IgnModelsToIds::CalcIdsFromModelVMessage).get_index();
}

void IgnModelsToIds::CalcIdsFromModelVMessage(const drake::systems::Context<double>& context,
                                              ignition::msgs::UInt32_V* output_ids) const {
  // Retrieves models and states inputs.
  const ignition::msgs::Model_V* input_models =
      this->template EvalInputValue<ignition::msgs::Model_V>(context, models_input_port_index_);

  output_ids->Clear();

  for (const auto& model : input_models->models()) {
    output_ids->add_data(model.id());
  }
}

}  // namespace delphyne
