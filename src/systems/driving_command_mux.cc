// Copyright 2018 Toyota Research Institute

#include "systems/driving_command_mux.h"

#include <numeric>
#include <utility>

#include <drake/common/default_scalars.h>

namespace delphyne {

template <typename T>
DrivingCommandMux<T>::DrivingCommandMux()
    : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<DrivingCommandMux>{}),
      steering_port_index_(this->DeclareInputPort(drake::systems::kVectorValued, 1).get_index()),
      acceleration_port_index_(this->DeclareInputPort(drake::systems::kVectorValued, 1).get_index()) {
  this->DeclareVectorOutputPort(DrivingCommand<T>(), &DrivingCommandMux<T>::CombineInputsToOutput);
}

template <typename T>
template <typename U>
DrivingCommandMux<T>::DrivingCommandMux(const DrivingCommandMux<U>&) : DrivingCommandMux<T>() {}

template <typename T>
const drake::systems::InputPort<T>& DrivingCommandMux<T>::steering_input() const {
  return drake::systems::System<T>::get_input_port(steering_port_index_);
}

template <typename T>
const drake::systems::InputPort<T>& DrivingCommandMux<T>::acceleration_input() const {
  return drake::systems::System<T>::get_input_port(acceleration_port_index_);
}

template <typename T>
void DrivingCommandMux<T>::CombineInputsToOutput(const drake::systems::Context<T>& context,
                                                 DrivingCommand<T>* output) const {
  const drake::systems::BasicVector<T>* steering = this->EvalVectorInput(context, steering_port_index_);
  DRAKE_DEMAND(steering->size() == 1);
  output->set_steering_angle(steering->GetAtIndex(0));
  const drake::systems::BasicVector<T>* acceleration = this->EvalVectorInput(context, acceleration_port_index_);
  DRAKE_DEMAND(acceleration->size() == 1);
  output->set_acceleration(acceleration->GetAtIndex(0));
}

}  // namespace delphyne

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::delphyne::DrivingCommandMux)
