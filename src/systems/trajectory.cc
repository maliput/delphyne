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

#include "systems/trajectory.h"

#include <algorithm>

namespace delphyne {

namespace {

// Helper to construct a vector of Eigen::MatrixXd from a vector of
// Eigen::Vector3d.
std::vector<Eigen::MatrixXd> ToVectorOfMatrixXd(const std::vector<Eigen::Vector3d>& translations) {
  std::vector<Eigen::MatrixXd> result{};
  for (const auto& translation : translations) {
    result.emplace_back(translation);
  }
  return result;
}

}  // namespace

using drake::multibody::SpatialVelocity;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;

PoseVelocity::PoseVelocity()
    : PoseVelocity::PoseVelocity(Eigen::Quaternion<double>::Identity(), Eigen::Vector3d::Zero(),
                                 SpatialVelocity<double>(drake::Vector6<double>::Zero())) {}

PoseVelocity::PoseVelocity(const Eigen::Quaternion<double>& rotation, const Eigen::Vector3d& translation,
                           const drake::multibody::SpatialVelocity<double>& velocity)
    : rotation_(rotation), translation_(translation), velocity_(velocity) {}

PoseVelocity Trajectory::value(double time) const {
  const Eigen::Quaternion<double> rotation(rotation_.orientation(time));
  const Eigen::Vector3d translation(translation_.value(time));
  const Eigen::Vector3d translation_dot(translation_dot_.value(time));
  const Eigen::Vector3d rpy_dot(rotation_.angular_velocity(time));
  const SpatialVelocity<double> velocity(rpy_dot, translation_dot);
  return PoseVelocity{rotation, translation, velocity};
}

Trajectory Trajectory::Make(const std::vector<double>& times,
                            const std::vector<Eigen::Quaternion<double>>& knots_rotation,
                            const std::vector<Eigen::Vector3d>& knots_translation,
                            const InterpolationType& interp_type) {
  DRAKE_THROW_UNLESS(times.size() == knots_rotation.size());
  DRAKE_THROW_UNLESS(times.size() == knots_translation.size());
  const PiecewiseQuaternionSlerp<double> rotation(times, knots_rotation);
  PiecewisePolynomial<double> translation;
  switch (interp_type) {
    case InterpolationType::kFirstOrderHold:
      translation = PiecewisePolynomial<double>::FirstOrderHold(times, ToVectorOfMatrixXd(knots_translation));
      break;
    case InterpolationType::kCubic:
      translation = PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          times, ToVectorOfMatrixXd(knots_translation));
      break;
    case InterpolationType::kPchip:
      translation = PiecewisePolynomial<double>::CubicShapePreserving(times, ToVectorOfMatrixXd(knots_translation));
      break;
    default:
      throw std::logic_error("The provided interp_type is not supported.");
  }
  return Trajectory{translation, rotation};
}

Trajectory Trajectory::MakeCubicFromWaypoints(const std::vector<Eigen::Quaternion<double>>& waypoints_rotation,
                                              const std::vector<Eigen::Vector3d>& waypoints_translation,
                                              const std::vector<double>& speeds) {
  DRAKE_THROW_UNLESS(!waypoints_rotation.empty());
  DRAKE_THROW_UNLESS(!waypoints_translation.empty());
  DRAKE_THROW_UNLESS(speeds.size() == waypoints_rotation.size());
  DRAKE_THROW_UNLESS(speeds.size() == waypoints_translation.size());
  std::vector<double> times(waypoints_rotation.size());
  std::vector<Eigen::MatrixXd> translations = ToVectorOfMatrixXd(waypoints_translation);
  times[0] = 0.;
  // Populate the segment times given a piecewise-linear travel time estimate.
  for (int i{0}; i < static_cast<int>(speeds.size()) - 1; i++) {
    DRAKE_THROW_UNLESS(speeds[i] >= 0.);
    // speed_k == 0. ⇒ speed_k+1 > 0., ∀ k = 0..N-1
    DRAKE_THROW_UNLESS(speeds[i + 1] > 0. || speeds[i] != 0.);
    const double distance = (translations[i] - translations[i + 1]).norm();
    const double average_speed = 0.5 * (speeds[i] + speeds[i + 1]);
    const double delta_t = distance / average_speed;
    times[i + 1] = delta_t + times[i];
  }
  const PiecewiseQuaternionSlerp<double> rotation(times, waypoints_rotation);

  // Starting with a piecewise-linear estimate of spline segment lengths, make
  // a loop that refines the segment lengths based on the constructed spline,
  // iterating until a tolerance is met.
  std::vector<Eigen::MatrixXd> linear_velocities(times.size());
  for (int i{0}; i < static_cast<int>(times.size()); i++) {
    const Eigen::Matrix3d rotation_matrix = drake::math::RotationMatrix<double>(waypoints_rotation[i]).matrix();
    // Represent forward speed in frame A as velocity in frame W.
    linear_velocities[i] = rotation_matrix * Eigen::Vector3d{speeds[i], 0., 0.};
  }
  const PiecewisePolynomial<double> translation =
      PiecewisePolynomial<double>::CubicHermite(times, translations, linear_velocities);

  return Trajectory(translation, rotation);
}

Trajectory Trajectory::MakeCubicFromWaypoints(const std::vector<Eigen::Quaternion<double>>& waypoints_rotation,
                                              const std::vector<Eigen::Vector3d>& waypoints_translation, double speed) {
  DRAKE_THROW_UNLESS(waypoints_rotation.size() == waypoints_translation.size());
  DRAKE_THROW_UNLESS(speed > 0.);
  std::vector<double> speeds(waypoints_rotation.size());
  std::fill(speeds.begin(), speeds.end(), speed);
  return MakeCubicFromWaypoints(waypoints_rotation, waypoints_translation, speeds);
}

Trajectory::Trajectory(const PiecewisePolynomial<double>& translation, const PiecewiseQuaternionSlerp<double>& rotation)
    : translation_(translation), rotation_(rotation), translation_dot_(translation.derivative()) {}

}  // namespace delphyne
