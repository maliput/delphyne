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

#pragma once

#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>

#include "gen/driving_command.h"

namespace delphyne {

/// A special-purpose multiplexer that packs two scalar inputs, steering angle
/// (in units rad) and acceleration (in units m/s^2), into a vector-valued
/// output of type DrivingCommand<T>, where the inputs feed directly through to
/// the output.
///
/// This class differs from systems::Multiplexer<T> constructed with a
/// DrivingCommand<T> model vector because a BasicVector (and notably any of its
/// subclasses) stored as T=double cannot yet be converted to another type.  See
/// #8921.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following `T` values are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// Currently, no other values for `T` are supported.
///
/// @ingroup automotive_systems
template <typename T>
class DrivingCommandMux : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrivingCommandMux)

  /// Constructs a %DrivingCommandMux with two scalar-valued input ports, and
  /// one output port containing a DrivingCommand<T>.
  DrivingCommandMux();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit DrivingCommandMux(const DrivingCommandMux<U>&);

  /// See the class description for details on the following input ports.
  /// @{
  const drake::systems::InputPort<T>& steering_input() const;
  const drake::systems::InputPort<T>& acceleration_input() const;
  /// @}

 private:
  // Packs a DrivingCommand based on the values seen at the input ports.
  void CombineInputsToOutput(const drake::systems::Context<T>& context, DrivingCommand<T>* output) const;

  const int steering_port_index_{};
  const int acceleration_port_index_{};
};

}  // namespace delphyne
