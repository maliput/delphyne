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

#include <memory>
#include <vector>

#include "backend/ign_publisher_system.h"
#include "backend/input_port_to_ign_converter.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

using drake::systems::VectorBase;

/// This class is a specialization of InputPortToIgnConverter that handles
/// only VectorBase input ports. Concrete subclasses only need to define
/// the vectorToIgn method, which does the actual conversion from the
/// input vector to an ignition message.
///
/// @tparam IGN_TYPE must be a valid ignition message type
template <class IGN_TYPE>
class VectorToIgnConverter : public InputPortToIgnConverter<IGN_TYPE> {
 public:
  /// Default constructor.
  ///
  /// @param[in] size The size of the vector to declare the input port.
  explicit VectorToIgnConverter(int size) : size_(size) {}

  /// Declare the input port for an IgnPublisherSystem. Since this class
  /// takes data from vector input ports, use kVectorValued as the port
  /// data type.
  ///
  /// @param[in] publisher The publisher for which we should define the port
  void DeclareInputPort(IgnPublisherSystem<IGN_TYPE>* publisher) override {
    DELPHYNE_DEMAND(publisher != nullptr);

    publisher->DeclareInputPort(drake::systems::kVectorValued, size_);
  }

  void ProcessInput(const IgnPublisherSystem<IGN_TYPE>* publisher,
                    const drake::systems::Context<double>& context,
                    int port_index, IGN_TYPE* ign_message) override {
    DELPHYNE_DEMAND(publisher != nullptr);
    DELPHYNE_DEMAND(ign_message != nullptr);

    const VectorBase<double>* const input_vector =
        publisher->EvalVectorInput(context, port_index);
    vectorToIgn(*input_vector, context.get_time(), ign_message);
  }

 protected:
  /*
   * Do the actual conversion from the input vector to the ignition message.
   *
   * @param[in] input_vector The vector retrieved from the input port.
   *
   * @param[in] time The current simulation time.
   *
   * @param[out] ign_message The ignition message, populated with the values
   * from the input vector.
   */
  virtual void vectorToIgn(const VectorBase<double>& input_vector, double time,
                           IGN_TYPE* ign_message) = 0;

  // The size of the vector in the VectorBase.
  const int size_;
};

}  // namespace backend
}  // namespace delphyne
