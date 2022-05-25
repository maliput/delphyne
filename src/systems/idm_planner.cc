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

#include "systems/idm_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <drake/common/autodiff.h>
#include <drake/common/cond.h>
#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>
#include <drake/common/extract_double.h>

namespace delphyne {

template <typename T>
const T IdmPlanner<T>::Evaluate(const IdmPlannerParameters<T>& params, const T& ego_velocity, const T& target_distance,
                                const T& target_distance_dot) {
  DRAKE_DEMAND(params.IsValid());

  using std::max;
  using std::pow;
  using std::sqrt;

  const T& v_ref = params.v_ref();
  const T& a = params.a();
  const T& b = params.b();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();

  DRAKE_DEMAND(a > 0.);
  DRAKE_DEMAND(b > 0.);
  DRAKE_DEMAND(target_distance > 0.);

  // Compute the interaction acceleration terms.
  const T& closing_term = ego_velocity * target_distance_dot / (2 * sqrt(a * b));
  const T& too_close_term = s_0 + ego_velocity * time_headway;
  const T& accel_interaction = drake::cond(target_distance < std::numeric_limits<T>::infinity(),
                                           pow((closing_term + too_close_term) / target_distance, 2.), T(0.));

  // Compute the free-road acceleration term.
  const T accel_free_road = pow(max(T(0.), ego_velocity) / v_ref, delta);

  // Compute the resultant acceleration (IDM equation).
  return a * (1. - accel_free_road - accel_interaction);
}

}  // namespace delphyne

// These instantiations must match the API documentation in idm_planner.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::delphyne::IdmPlanner)
