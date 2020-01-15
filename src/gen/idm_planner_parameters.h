// Copyright 2018 Toyota Research Institute

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <drake/common/drake_bool.h>
#include <drake/common/dummy_value.h>
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>

#include <maliput/common/maliput_never_destroyed.h>

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

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

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  IdmPlannerParameters(const IdmPlannerParameters& other) : drake::systems::BasicVector<T>(other.values()) {}
  IdmPlannerParameters(IdmPlannerParameters&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  IdmPlannerParameters& operator=(const IdmPlannerParameters& other) {
    this->values() = other.values();
    return *this;
  }
  IdmPlannerParameters& operator=(IdmPlannerParameters&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a drake::symbolic::Variable for each element with the known
  /// variable
  /// name.  This is only available for T == drake::symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, drake::symbolic::Expression>::value>::type SetToNamedVariables() {
    this->set_v_ref(drake::symbolic::Variable("v_ref"));
    this->set_a(drake::symbolic::Variable("a"));
    this->set_b(drake::symbolic::Variable("b"));
    this->set_s_0(drake::symbolic::Variable("s_0"));
    this->set_time_headway(drake::symbolic::Variable("time_headway"));
    this->set_delta(drake::symbolic::Variable("delta"));
    this->set_bloat_diameter(drake::symbolic::Variable("bloat_diameter"));
    this->set_distance_lower_limit(drake::symbolic::Variable("distance_lower_limit"));
    this->set_scan_ahead_distance(drake::symbolic::Variable("scan_ahead_distance"));
  }

  IdmPlannerParameters<T>* DoClone() const final { return new IdmPlannerParameters; }

  /// @name Getters and Setters
  //@{
  /// desired velocity in free traffic
  /// @note @c v_ref is expressed in units of m/s.
  /// @note @c v_ref has a limited domain of [0.0, +Inf].
  const T& v_ref() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kVRef);
  }
  /// Setter that matches v_ref().
  void set_v_ref(const T& v_ref) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kVRef, v_ref);
  }
  /// Fluent setter that matches v_ref().
  /// Returns a copy of `this` with v_ref set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_v_ref(const T& v_ref) const {
    IdmPlannerParameters<T> result(*this);
    result.set_v_ref(v_ref);
    return result;
  }
  /// max acceleration
  /// @note @c a is expressed in units of m/s^2.
  /// @note @c a has a limited domain of [0.0, +Inf].
  const T& a() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kA);
  }
  /// Setter that matches a().
  void set_a(const T& a) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kA, a);
  }
  /// Fluent setter that matches a().
  /// Returns a copy of `this` with a set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_a(const T& a) const {
    IdmPlannerParameters<T> result(*this);
    result.set_a(a);
    return result;
  }
  /// comfortable braking deceleration
  /// @note @c b is expressed in units of m/s^2.
  /// @note @c b has a limited domain of [0.0, +Inf].
  const T& b() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kB);
  }
  /// Setter that matches b().
  void set_b(const T& b) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kB, b);
  }
  /// Fluent setter that matches b().
  /// Returns a copy of `this` with b set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_b(const T& b) const {
    IdmPlannerParameters<T> result(*this);
    result.set_b(b);
    return result;
  }
  /// minimum desired net distance
  /// @note @c s_0 is expressed in units of m.
  /// @note @c s_0 has a limited domain of [0.0, +Inf].
  const T& s_0() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kS0);
  }
  /// Setter that matches s_0().
  void set_s_0(const T& s_0) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kS0, s_0);
  }
  /// Fluent setter that matches s_0().
  /// Returns a copy of `this` with s_0 set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_s_0(const T& s_0) const {
    IdmPlannerParameters<T> result(*this);
    result.set_s_0(s_0);
    return result;
  }
  /// desired time headway to vehicle in front
  /// @note @c time_headway is expressed in units of s.
  /// @note @c time_headway has a limited domain of [0.0, +Inf].
  const T& time_headway() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTimeHeadway);
  }
  /// Setter that matches time_headway().
  void set_time_headway(const T& time_headway) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTimeHeadway, time_headway);
  }
  /// Fluent setter that matches time_headway().
  /// Returns a copy of `this` with time_headway set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_time_headway(const T& time_headway) const {
    IdmPlannerParameters<T> result(*this);
    result.set_time_headway(time_headway);
    return result;
  }
  /// free-road exponent
  /// @note @c delta is expressed in units of dimensionless.
  /// @note @c delta has a limited domain of [0.0, +Inf].
  const T& delta() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kDelta);
  }
  /// Setter that matches delta().
  void set_delta(const T& delta) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kDelta, delta);
  }
  /// Fluent setter that matches delta().
  /// Returns a copy of `this` with delta set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_delta(const T& delta) const {
    IdmPlannerParameters<T> result(*this);
    result.set_delta(delta);
    return result;
  }
  /// diameter of circle about the vehicle's pose that encloses its physical
  /// footprint
  /// @note @c bloat_diameter is expressed in units of m.
  /// @note @c bloat_diameter has a limited domain of [0.0, +Inf].
  const T& bloat_diameter() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kBloatDiameter);
  }
  /// Setter that matches bloat_diameter().
  void set_bloat_diameter(const T& bloat_diameter) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kBloatDiameter, bloat_diameter);
  }
  /// Fluent setter that matches bloat_diameter().
  /// Returns a copy of `this` with bloat_diameter set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_bloat_diameter(const T& bloat_diameter) const {
    IdmPlannerParameters<T> result(*this);
    result.set_bloat_diameter(bloat_diameter);
    return result;
  }
  /// lower saturation bound on net distance to prevent near-singular IDM
  /// solutions
  /// @note @c distance_lower_limit is expressed in units of m.
  /// @note @c distance_lower_limit has a limited domain of [0.0, +Inf].
  const T& distance_lower_limit() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kDistanceLowerLimit);
  }
  /// Setter that matches distance_lower_limit().
  void set_distance_lower_limit(const T& distance_lower_limit) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kDistanceLowerLimit, distance_lower_limit);
  }
  /// Fluent setter that matches distance_lower_limit().
  /// Returns a copy of `this` with distance_lower_limit set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_distance_lower_limit(const T& distance_lower_limit) const {
    IdmPlannerParameters<T> result(*this);
    result.set_distance_lower_limit(distance_lower_limit);
    return result;
  }
  /// distance to scan ahead on road for a leading vehicle
  /// @note @c scan_ahead_distance is expressed in units of m.
  /// @note @c scan_ahead_distance has a limited domain of [0.0, +Inf].
  const T& scan_ahead_distance() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kScanAheadDistance);
  }
  /// Setter that matches scan_ahead_distance().
  void set_scan_ahead_distance(const T& scan_ahead_distance) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kScanAheadDistance, scan_ahead_distance);
  }
  /// Fluent setter that matches scan_ahead_distance().
  /// Returns a copy of `this` with scan_ahead_distance set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  IdmPlannerParameters<T> with_scan_ahead_distance(const T& scan_ahead_distance) const {
    IdmPlannerParameters<T> result(*this);
    result.set_scan_ahead_distance(scan_ahead_distance);
    return result;
  }
  //@}

  /// See IdmPlannerParametersIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return IdmPlannerParametersIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
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

  void GetElementBounds(Eigen::VectorXd* lower, Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 9, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 9, 1>::Constant(kInf);
    (*lower)(K::kVRef) = 0.0;
    (*lower)(K::kA) = 0.0;
    (*lower)(K::kB) = 0.0;
    (*lower)(K::kS0) = 0.0;
    (*lower)(K::kTimeHeadway) = 0.0;
    (*lower)(K::kDelta) = 0.0;
    (*lower)(K::kBloatDiameter) = 0.0;
    (*lower)(K::kDistanceLowerLimit) = 0.0;
    (*lower)(K::kScanAheadDistance) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The IdmPlannerParameters vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
