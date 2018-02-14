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

#include "drake/systems/framework/context.h"

using drake::systems::AbstractValue;
using drake::systems::BasicVector;
using drake::systems::VectorBase;

namespace delphyne {
namespace backend {

template <class IGN_TYPE>
class IgnPublisherSystem;

template <class IGN_TYPE>
class IgnSubscriberSystem;

/// Is in charge of converting the data of an input port to an ignition message.
/// Since the converted knows what type of input port it works on it also
/// has the responsibility of defining it.
template <class IGN_TYPE>
class IgnitionMessageConverter {
 public:
  /// Default constructor.
  IgnitionMessageConverter() = default;

  /// Indicates if this converter uses discrete (VectorBased) values or not.
  /// Must be overridden by subclasses.
  ///
  /// @return A boolean stating if this converter uses discrete (vector-based)
  /// values.
  virtual bool handles_discrete_values() = 0;

  /// Allocates the object that will be used as a value in the output port.
  /// Returns nullptr if the subscriber handles abstract values.
  /// By default, return nullptr and let the subclasses override when
  /// appropriate.
  ///
  /// @return A pointer to a BasicVector object that will be used as the output
  /// value or a nullptr if this converter doesn't handle discrete values.
  virtual std::unique_ptr<BasicVector<double>> AllocateDiscreteOutputValue()
      const {
    return nullptr;
  }

  /// Allocates the object that will be used as a value in the output port.
  /// Returns nullptr if the subscriber handles discrete values.
  /// By default, return nullptr and let the subclasses override when
  /// appropriate.
  ///
  /// @return A pointer to an AbstractValue object that will be used as the
  /// output value or a nullptr if this converter doesn't handle abstract
  /// values.
  virtual std::unique_ptr<AbstractValue> AllocateAbstractDefaultValue() const {
    return nullptr;
  }

  /// Given an @p ign_message received in the topic we are listening to, convert
  /// it to a discrete value to be sent to the output port.
  /// This is a no-op by default, subclasses should override when appropriate.
  ///
  /// @param[in] ign_message The object received in the ignition channel
  /// that we are subscribed to,
  ///
  /// @param[out] output_vector The vector we should fill with @p ign_message
  /// data to be sent to the output port.
  virtual void ProcessDiscreteOutput(const IGN_TYPE& ign_message,
                                     VectorBase<double>* output_vector) {}

  /// Given an @p ign_message received in the topic we are listening to, convert
  /// it to an abstract value to be sent to the output port.
  /// This is a no-op by default, subclasses should override when appropriate.
  ///
  /// @param[in] ign_message The object received in the ignition channel
  /// that we are subscribed to.
  ///
  /// @param[out] output_vector The vector we should fill with @p ign_message
  /// data to be sent to the output port.
  virtual void ProcessAbstractOutput(const IGN_TYPE& ign_message,
                                     AbstractValue* output_value) {}

  /// Get the data from the input port and populate the outgoing ignition
  /// message based on it. Subclasses must override this.
  ///
  /// @param[in] publisher The publisher for which we are filling the
  /// @p ign_message.
  ///
  /// @param[in] context The simulation context.
  ///
  /// @param[in] port_index The index of the port the converter must read from.
  ///
  /// @param[out] ign_message The ignition message to populate.
  virtual void ProcessInput(const IgnPublisherSystem<IGN_TYPE>* publisher,
                            const drake::systems::Context<double>& context,
                            int port_index, IGN_TYPE* ign_message) = 0;

  /// @returns The size of the vector-based object.
  virtual int get_vector_size() = 0;
};

}  // namespace backend
}  // namespace delphyne
