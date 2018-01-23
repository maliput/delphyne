// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "drake/systems/framework/context.h"

#include "bridge/lcm_to_ign_translation.h"

#include "backend/ign_publisher_system.h"
#include "backend/input_port_to_ign_converter.h"

using drake::systems::AbstractValue;

namespace delphyne {
namespace backend {

/// This class is a specialization of InputPortToIgnConverter that handles
/// abstract input ports. The input values are then casted to the
/// LCM_TYPE template parameter and further converted to an ignition message.
///
/// @tparam LCM_TYPE must be a valid LCM message type
///
/// @tparam IGN_TYPE must be a valid ignition message type
template <class LCM_TYPE, class IGN_TYPE>
class AbstractInputToIgnConverter : public InputPortToIgnConverter<IGN_TYPE> {
 public:
  /// Declares the input port for an IgnPublisherSystem. Since this class
  /// takes data from abstract input ports, uses DeclareAbstractInputPort to
  /// define it.
  ///
  /// @param[in] publisher The publisher for which we should define the port
  void DeclareInputPort(IgnPublisherSystem<IGN_TYPE>* publisher) override {
    publisher->DeclareAbstractInputPort();
  }

  void ProcessInput(const IgnPublisherSystem<IGN_TYPE>* publisher,
                    const drake::systems::Context<double>& context,
                    int port_index, IGN_TYPE* ign_message) override {
    const AbstractValue* input =
        publisher->EvalAbstractInput(context, port_index);
    const LCM_TYPE lcm_msg = input->GetValue<LCM_TYPE>();
    delphyne::bridge::lcmToIgn(lcm_msg, ign_message);
  }
};

}  // namespace backend
}  // namespace delphyne
