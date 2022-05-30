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

#include <memory>

#include <Eigen/Geometry>
#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/scalar_conversion_traits.h>
#include <drake/systems/rendering/pose_vector.h>

#include "gen/pure_pursuit_params.h"
#include "gen/simple_car_params.h"
#include "systems/lane_direction.h"

namespace delphyne {

/// PurePursuitController implements a pure pursuit controller.  See PurePursuit
/// for details on the approach.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: a LaneDirection representing the requested lane and direction
///   of travel.
///   (InputPort getter: lane_input())
///
/// Input Port 1: PoseVector for the ego car.
///   (InputPort getter: ego_pose_input())
///
/// Output Port 0: A BasicVector of size one with the commanded steering angle.
///   (OutputPort getter: steering_command_output())
///
/// @ingroup automotive_controllers
template <typename T>
class PurePursuitController : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuitController)

  /// Constructor.
  PurePursuitController();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit PurePursuitController(const PurePursuitController<U>&) : PurePursuitController<T>() {}

  ~PurePursuitController() override;

  /// Returns the port to the individual input/output ports.
  const drake::systems::InputPort<T>& lane_input() const;
  const drake::systems::InputPort<T>& ego_pose_input() const;
  const drake::systems::OutputPort<T>& steering_command_output() const;

 private:
  void OutputSteeringCommand(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;

  void CalcSteeringCommand(const PurePursuitParams<T>& pp_params, const SimpleCarParams<T>& car_params,
                           const LaneDirection& lane_direction,
                           const drake::systems::rendering::PoseVector<T>& ego_pose,
                           drake::systems::BasicVector<T>* command) const;

  // Indices for the input / output ports.
  const int lane_index_{};
  const int ego_pose_index_{};
  const int steering_command_index_{};
};

}  // namespace delphyne

namespace drake {
namespace systems {
namespace scalar_conversion {
// Disables symbolic support, because maliput's LanePosition <-> InertialPosition
// conversion (used in pure_pursuit.cc) is not symbolic-supported.
template <>
struct Traits<::delphyne::PurePursuitController> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
