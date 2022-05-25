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

#include "systems/calc_smooth_acceleration.h"

#include <algorithm>
#include <string>

#include <drake/common/autodiff.h>
#include <drake/common/cond.h>
#include <drake/common/symbolic.h>

namespace delphyne {

using drake::AutoDiffXd;
using drake::cond;
using drake::symbolic::Expression;

template <typename T>
T calc_smooth_acceleration(const T& desired_acceleration, const T& max_velocity, const T& velocity_limit_kp,
                           const T& current_velocity) {
  using std::max;
  using std::min;
  using std::tanh;

  // If our current velocity is out of bounds, insist on damping that brings us
  // back toward the limit, but allow for the desired_acceleration to win if it
  // is stronger than the damping and has the desired sign.
  const T underspeed = 0 - current_velocity;
  const T overspeed = current_velocity - max_velocity;
  const T damped_acceleration = cond(
      // If velocity is too low, use positive damping or desired_acceleration.
      underspeed > 0, max(desired_acceleration, T(velocity_limit_kp * underspeed)),
      // If velocity is too high, use negative damping or desired_acceleration.
      overspeed > 0, min(desired_acceleration, T(-velocity_limit_kp * overspeed)),
      // Velocity is within limits.
      desired_acceleration);

  // TODO(jwnimmer-tri) Declare witness functions for the above conditions,
  // once the framework support is in place.  Until then, smooth out the
  // acceleration using tanh centered around the limit we are headed towards
  // (max speed when accelerating; zero when decelerating).  The smoothing
  // constant within the tanh is arbitrary and un-tuned.
  const T relevant_limit = cond(damped_acceleration >= 0, max_velocity, T(0));
  const T smoothing_factor = pow(tanh(20.0 * (current_velocity - relevant_limit)), 2);
  return damped_acceleration * smoothing_factor;
}

// These instantiations must match the API documentation in
// calc_smooth_acceleration.h.
template double calc_smooth_acceleration<double>(const double& desired_acceleration, const double& current_velocity,
                                                 const double& max_velocity, const double& velocity_limit_kp);
template AutoDiffXd calc_smooth_acceleration<AutoDiffXd>(const AutoDiffXd& desired_acceleration,
                                                         const AutoDiffXd& current_velocity,
                                                         const AutoDiffXd& max_velocity,
                                                         const AutoDiffXd& velocity_limit_kp);
template Expression calc_smooth_acceleration<Expression>(const Expression& desired_acceleration,
                                                         const Expression& current_velocity,
                                                         const Expression& max_velocity,
                                                         const Expression& velocity_limit_kp);

}  // namespace delphyne
