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

#include "systems/road_path.h"

#include <vector>

#include <drake/common/cond.h>
#include <drake/common/drake_assert.h>
#include <drake/math/saturate.h>
#include <maliput/api/branch_point.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/maliput_unused.h>

#include "delphyne/macros.h"

namespace delphyne {

using drake::MatrixX;
using drake::Vector3;
using drake::trajectories::PiecewisePolynomial;
using maliput::api::InertialPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::RoadGeometry;

template <typename T>
RoadPath<T>::RoadPath(const LaneDirection& initial_lane_direction, const T& step_size, int num_breaks)
    : path_(MakePiecewisePolynomial(initial_lane_direction, step_size, num_breaks)),
      path_prime_(path_.derivative(1 /* 1st derivative */)),
      path_double_prime_(path_.derivative(2 /* 2nd derivative */)) {}

template <typename T>
RoadPath<T>::~RoadPath() {}

template <typename T>
const PiecewisePolynomial<T>& RoadPath<T>::get_path() const {
  return path_;
}

template <typename T>
const T RoadPath<T>::GetClosestPathPosition(const Vector3<T>& inertial_pos, const T& s_guess) const {
  maliput::common::unused(inertial_pos, s_guess);
  DELPHYNE_ABORT_MESSAGE("Unused.");
  // Return statement to silence -Wreturn-type warning.
  return T(0);
}

template <typename T>
const PiecewisePolynomial<T> RoadPath<T>::MakePiecewisePolynomial(const LaneDirection& initial_lane_direction,
                                                                  const T& step_size, int num_breaks) const {
  std::vector<T> s_breaks{};
  std::vector<MatrixX<T>> inertial_knots(num_breaks, MatrixX<T>::Zero(3, 1));

  LaneDirection ld = initial_lane_direction;
  T s_lane{drake::cond(ld.with_s, T(0.), T(ld.lane->length()))};
  T s_break{0.};

  // Loop over all the breaks and extract the knot points.
  for (int i = 0; i < num_breaks - 1; ++i) {
    s_breaks.emplace_back(s_break);
    s_break += T(step_size);

    InertialPosition inertial_pos = ld.lane->ToInertialPosition({s_lane /* s */, 0. /* r */, 0. /* h */});
    inertial_knots[i] << T(inertial_pos.x()), T(inertial_pos.y()), T(inertial_pos.z());

    // Take a step.
    if (ld.with_s) {
      s_lane += T(step_size);
    } else {
      s_lane -= T(step_size);
    }

    // Compute the distance in excess of the lane boundary by taking this step.
    const T out_distance{drake::cond(ld.with_s, T(s_lane - ld.lane->length()), T(-s_lane))};
    if (out_distance >= 0.) {
      const LaneEndSet* lane_end_set{ld.with_s ? ld.lane->GetOngoingBranches(LaneEnd::kFinish)
                                               : ld.lane->GetOngoingBranches(LaneEnd::kStart)};
      if (lane_end_set->size() == 0) {  // There are no more ongoing lanes.
        // If needed, add another knot point to make up the remaining distance.
        if (out_distance != 0.) {
          s_breaks.emplace_back(s_break + T(step_size - out_distance));
          s_lane = drake::cond(ld.with_s, T(ld.lane->length()), T(0.));
          inertial_pos = ld.lane->ToInertialPosition({s_lane /* s */, 0. /* r */, 0. /* h */});
          inertial_knots[i + 1] << T(inertial_pos.x()), T(inertial_pos.y()), T(inertial_pos.z());
        }
        break;
      }

      // Always choose the first lane in the set as the new lane.
      ld.lane = lane_end_set->get(0).lane;
      ld.with_s = (lane_end_set->get(0).end == LaneEnd::kStart) ? true : false;

      // Correct for the distance overshoot.
      s_lane = drake::cond(ld.with_s, out_distance, T(ld.lane->length()) - out_distance);
    }
  }
  // Resize the vector of knot points, if necessary.
  inertial_knots.resize(s_breaks.size());

  // Create the resulting piecewise polynomial.
  return PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(s_breaks, inertial_knots);
}

template class RoadPath<double>;

}  // namespace delphyne
