// Copyright 2017 Open Source Robotics Foundation
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

#include <memory>
#include <vector>

namespace delphyne {
namespace backend {

template <class IGN_TYPE>
class IgnPublisherSystem;

/// Is in charge of converting the data of an input port to an ignition message.
/// Since the converted knows what type of input port it works on it also
/// has the responsibility of defining it.
template <class IGN_TYPE>
class InputPortToIgnConverter {
 public:
  /// Default constructor.
  InputPortToIgnConverter() = default;

  /// Declare the input port for an IgnPublisherSystem. Subclasses must
  /// override this.
  ///
  /// @param[in] publisher The publisher for which we should define the port
  virtual void declareInputPort(IgnPublisherSystem<IGN_TYPE>* publisher) = 0;

  /// Get the data from the input port and populate the outgoing ignition
  /// message based on it. Subclasses must override this.
  ///
  /// @param[in] publisher The publisher for which we are filling the message.
  ///
  /// @param[in] context The simulation context.
  ///
  /// @param[in] port_index The index of the port the converter must read from.
  ///
  /// @param[out] ign_message The ignition message to populate
  virtual void processInput(const IgnPublisherSystem<IGN_TYPE>* publisher,
                            const drake::systems::Context<double>& context,
                            int port_index, IGN_TYPE* ign_message) = 0;
};

}  // namespace backend
}  // namespace delphyne
