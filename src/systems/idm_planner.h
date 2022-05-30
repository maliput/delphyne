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

#include "gen/idm_planner_parameters.h"

namespace delphyne {

/// IdmPlanner implements the IDM (Intelligent Driver Model) equation governing
/// longitudinal accelerations of a vehicle in single-lane traffic [1, 2].  It
/// is derived based on qualitative observations of actual driving behavior and
/// captures objectives such as keeping a safe distance behind a lead vehicle,
/// maintaining a desired speed, and accelerating and decelerating within
/// comfortable limits.
///
/// The IDM equation produces accelerations that realize smooth transitions
/// between the following three modes:
///  - Free-road behavior: when the distance to the leading car is large, the
///    IDM regulates acceleration to match the desired speed `v_0`.
///  - Fast-closing-speed behavior: when the target distance decreases, an
///    interaction term compensates for the velocity difference, while keeping
///    deceleration comfortable according to parameter `b`.
///  - Small-distance behavior: within small net distances to the lead vehicle,
///    comfort is ignored in favor of increasing this distance to `s_0`.
///
/// See the corresponding .cc file for details about the IDM equation.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// [1] Martin Treiber and Arne Kesting. Traffic Flow Dynamics, Data, Models,
///     and Simulation. Springer, 2013.
///
/// [2] https://en.wikipedia.org/wiki/Intelligent_driver_model.
template <typename T>
class IdmPlanner {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmPlanner)
  IdmPlanner() = delete;

  /// Evaluates the IDM equation for the chosen planner parameters @p params,
  /// given the current velocity @p ego_velocity, distance to the lead car @p
  /// target_distance, and the closing velocity @p target_distance_dot.  The
  /// returned value is a longitudinal acceleration.
  static const T Evaluate(const IdmPlannerParameters<T>& params, const T& ego_velocity, const T& target_distance,
                          const T& target_distance_dot);
};

}  // namespace delphyne
