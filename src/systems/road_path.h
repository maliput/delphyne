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
#include <drake/common/eigen_types.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <maliput/api/road_geometry.h>

#include "systems/lane_direction.h"

namespace delphyne {

/// RoadPath converts a sequence of Maliput Lanes into a PiecewisePolynomial for
/// the purpose of generating a path for a car to follow.  The path is created
/// from the start of a user-specified initial lane and direction of travel, and
/// proceeds in that direction until either the end of the road is reached
/// (there exist no ongoing lanes), or the given number of specified breaks have
/// been traversed.  The resulting path is a cubic spline that matches the `r =
/// 0` coordinate of the lanes at each specified break point.  The resulting
/// piecewise curve is C2-continuous throughout with zero first and second
/// derivatives at the start and end of the path.
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double is supported.
template <typename T>
class RoadPath {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadPath)

  /// Constructs a single RoadPath from a sequence of Maliput lanes based on the
  /// following parameters:
  /// @param initial_lane_direction contains the initial LaneDirection.  This
  /// must be a valid @p road Lane.
  /// @param step_size is the size of each step (in the `s`-direction) between
  /// knot points.
  /// @param num_breaks are the number of breaks at which the knot points will
  /// be evaluated.
  RoadPath(const LaneDirection& initial_lane_direction, const T& step_size, int num_breaks);
  ~RoadPath();

  const drake::trajectories::PiecewisePolynomial<T>& get_path() const;

  /// Computes the closest `s`-position on the path to an arbitrary point in
  /// the world frame of the provided Maliput Lanes.  (Not yet implemented)
  // TODO(jadecastro): Implement this.
  const T GetClosestPathPosition(const drake::Vector3<T>& geo_position, const T& s_guess) const;

 private:
  // Traverse the road, starting from an initial LaneDirection, and build a
  // cubic spline PiecewisePolynomial until 1) a given a number of segments has
  // been traversed, or 2) the end of the road has been reached.
  //
  // If a BranchPoint is encountered in which there is more than one ongoing
  // lane, the zero-index lane is always selected.
  // TODO(jadecastro): Use Lane::GetDefaultBranch() to decide the ongoing Lane.
  const drake::trajectories::PiecewisePolynomial<T> MakePiecewisePolynomial(const LaneDirection& initial_lane_direction,
                                                                            const T& step_size, int num_breaks) const;

  // The path representing the mid-curve of the road.
  drake::trajectories::PiecewisePolynomial<T> path_;

  // First derivative of path_.
  drake::trajectories::PiecewisePolynomial<T> path_prime_;

  // Second derivative of path_.
  drake::trajectories::PiecewisePolynomial<T> path_double_prime_;
};

}  // namespace delphyne
