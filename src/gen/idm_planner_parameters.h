// Copyright 2018 Toyota Research Institute

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/dummy_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

namespace delphyne {

/// Describes the row indices of a IdmPlannerParameters.
struct IdmPlannerParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 9;

  // The index of each individual coordinate.
  static const int kVRef = 0;
  static const int kA = 1;
  static const int kB = 2;
  static const int kS0 = 3;
  static const int kTimeHeadway = 4;
  static const int kDelta = 5;
  static const int kBloatDiameter = 6;
  static const int kDistanceLowerLimit = 7;
  static const int kScanAheadDistance = 8;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `IdmPlannerParametersIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class IdmPlannerParameters final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef IdmPlannerParametersIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c v_ref defaults to 10.0 m/s.
  /// @arg @c a defaults to 1.0 m/s^2.
  /// @arg @c b defaults to 3.0 m/s^2.
  /// @arg @c s_0 defaults to 1.0 m.
  /// @arg @c time_headway defaults to 0.1 s.
  /// @arg @c delta defaults to 4.0 dimensionless.
  /// @arg @c bloat_diameter defaults to 4.5 m.
  /// @arg @c distance_lower_limit defaults to 1e-2 m.
  /// @arg @c scan_ahead_distance defaults to 100.0 m.
  IdmPlannerParameters() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_v_ref(10.0);
    this->set_a(1.0);
    this->set_b(3.0);
    this->set_s_0(1.0);
    this->set_time_headway(0.1);
    this->set_delta(4.0);
    this->set_bloat_diameter(4.5);
    this->set_distance_lower_limit(1e-2);
    this->set_scan_ahead_distance(100.0);
  }

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_v_ref(drake::symbolic::Variable("v_ref"));
    this->set_a(drake::symbolic::Variable("a"));
    this->set_b(drake::symbolic::Variable("b"));
    this->set_s_0(drake::symbolic::Variable("s_0"));
    this->set_time_headway(drake::symbolic::Variable("time_headway"));
    this->set_delta(drake::symbolic::Variable("delta"));
    this->set_bloat_diameter(drake::symbolic::Variable("bloat_diameter"));
    this->set_distance_lower_limit(
        drake::symbolic::Variable("distance_lower_limit"));
    this->set_scan_ahead_distance(
        drake::symbolic::Variable("scan_ahead_distance"));
  }

  IdmPlannerParameters<T>* DoClone() const final {
    return new IdmPlannerParameters;
  }

  /// @name Getters and Setters
  //@{
  /// desired velocity in free traffic
  /// @note @c v_ref is expressed in units of m/s.
  /// @note @c v_ref has a limited domain of [0.0, +Inf].
  const T& v_ref() const { return this->GetAtIndex(K::kVRef); }
  void set_v_ref(const T& v_ref) { this->SetAtIndex(K::kVRef, v_ref); }
  /// max acceleration
  /// @note @c a is expressed in units of m/s^2.
  /// @note @c a has a limited domain of [0.0, +Inf].
  const T& a() const { return this->GetAtIndex(K::kA); }
  void set_a(const T& a) { this->SetAtIndex(K::kA, a); }
  /// comfortable braking deceleration
  /// @note @c b is expressed in units of m/s^2.
  /// @note @c b has a limited domain of [0.0, +Inf].
  const T& b() const { return this->GetAtIndex(K::kB); }
  void set_b(const T& b) { this->SetAtIndex(K::kB, b); }
  /// minimum desired net distance
  /// @note @c s_0 is expressed in units of m.
  /// @note @c s_0 has a limited domain of [0.0, +Inf].
  const T& s_0() const { return this->GetAtIndex(K::kS0); }
  void set_s_0(const T& s_0) { this->SetAtIndex(K::kS0, s_0); }
  /// desired time headway to vehicle in front
  /// @note @c time_headway is expressed in units of s.
  /// @note @c time_headway has a limited domain of [0.0, +Inf].
  const T& time_headway() const { return this->GetAtIndex(K::kTimeHeadway); }
  void set_time_headway(const T& time_headway) {
    this->SetAtIndex(K::kTimeHeadway, time_headway);
  }
  /// free-road exponent
  /// @note @c delta is expressed in units of dimensionless.
  /// @note @c delta has a limited domain of [0.0, +Inf].
  const T& delta() const { return this->GetAtIndex(K::kDelta); }
  void set_delta(const T& delta) { this->SetAtIndex(K::kDelta, delta); }
  /// diameter of circle about the vehicle's pose that encloses its physical
  /// footprint
  /// @note @c bloat_diameter is expressed in units of m.
  /// @note @c bloat_diameter has a limited domain of [0.0, +Inf].
  const T& bloat_diameter() const {
    return this->GetAtIndex(K::kBloatDiameter);
  }
  void set_bloat_diameter(const T& bloat_diameter) {
    this->SetAtIndex(K::kBloatDiameter, bloat_diameter);
  }
  /// lower saturation bound on net distance to prevent near-singular IDM
  /// solutions
  /// @note @c distance_lower_limit is expressed in units of m.
  /// @note @c distance_lower_limit has a limited domain of [0.0, +Inf].
  const T& distance_lower_limit() const {
    return this->GetAtIndex(K::kDistanceLowerLimit);
  }
  void set_distance_lower_limit(const T& distance_lower_limit) {
    this->SetAtIndex(K::kDistanceLowerLimit, distance_lower_limit);
  }
  /// distance to scan ahead on road for a leading vehicle
  /// @note @c scan_ahead_distance is expressed in units of m.
  /// @note @c scan_ahead_distance has a limited domain of [0.0, +Inf].
  const T& scan_ahead_distance() const {
    return this->GetAtIndex(K::kScanAheadDistance);
  }
  void set_scan_ahead_distance(const T& scan_ahead_distance) {
    this->SetAtIndex(K::kScanAheadDistance, scan_ahead_distance);
  }
  //@}

  /// See IdmPlannerParametersIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return IdmPlannerParametersIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(v_ref());
    result = result && (v_ref() >= T(0.0));
    result = result && !isnan(a());
    result = result && (a() >= T(0.0));
    result = result && !isnan(b());
    result = result && (b() >= T(0.0));
    result = result && !isnan(s_0());
    result = result && (s_0() >= T(0.0));
    result = result && !isnan(time_headway());
    result = result && (time_headway() >= T(0.0));
    result = result && !isnan(delta());
    result = result && (delta() >= T(0.0));
    result = result && !isnan(bloat_diameter());
    result = result && (bloat_diameter() >= T(0.0));
    result = result && !isnan(distance_lower_limit());
    result = result && (distance_lower_limit() >= T(0.0));
    result = result && !isnan(scan_ahead_distance());
    result = result && (scan_ahead_distance() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(9);
    (*value)[0] = v_ref() - T(0.0);
    (*value)[1] = a() - T(0.0);
    (*value)[2] = b() - T(0.0);
    (*value)[3] = s_0() - T(0.0);
    (*value)[4] = time_headway() - T(0.0);
    (*value)[5] = delta() - T(0.0);
    (*value)[6] = bloat_diameter() - T(0.0);
    (*value)[7] = distance_lower_limit() - T(0.0);
    (*value)[8] = scan_ahead_distance() - T(0.0);
  }
};

}  // namespace delphyne
