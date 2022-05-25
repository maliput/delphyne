// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
