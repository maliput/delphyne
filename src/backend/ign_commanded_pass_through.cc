// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
