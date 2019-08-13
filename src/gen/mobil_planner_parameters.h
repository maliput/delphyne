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
#include <drake/common/never_destroyed.h>
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

namespace delphyne {

/// Describes the row indices of a MobilPlannerParameters.
struct MobilPlannerParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 3;

  // The index of each individual coordinate.
  static const int kP = 0;
  static const int kThreshold = 1;
  static const int kMaxDeceleration = 2;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `MobilPlannerParametersIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MobilPlannerParameters final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MobilPlannerParametersIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c p defaults to 0.5 dimensionless.
  /// @arg @c threshold defaults to 0.1 m/s^2.
  /// @arg @c max_deceleration defaults to 4.0 m/s^2.
  MobilPlannerParameters() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_p(0.5);
    this->set_threshold(0.1);
    this->set_max_deceleration(4.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  MobilPlannerParameters(const MobilPlannerParameters& other) : drake::systems::BasicVector<T>(other.values()) {}
  MobilPlannerParameters(MobilPlannerParameters&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  MobilPlannerParameters& operator=(const MobilPlannerParameters& other) {
    this->values() = other.values();
    return *this;
  }
  MobilPlannerParameters& operator=(MobilPlannerParameters&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, drake::symbolic::Expression>::value>::type SetToNamedVariables() {
    this->set_p(drake::symbolic::Variable("p"));
    this->set_threshold(drake::symbolic::Variable("threshold"));
    this->set_max_deceleration(drake::symbolic::Variable("max_deceleration"));
  }

  MobilPlannerParameters<T>* DoClone() const final { return new MobilPlannerParameters; }

  /// @name Getters and Setters
  //@{
  /// politeness factor (0.0 is purely egoistic, higher values increase
  /// politeness)
  /// @note @c p is expressed in units of dimensionless.
  /// @note @c p has a limited domain of [0.0, 1.0].
  const T& p() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kP);
  }
  /// Setter that matches p().
  void set_p(const T& p) { this->SetAtIndex(K::kP, p); }
  /// Fluent setter that matches p().
  /// Returns a copy of `this` with p set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MobilPlannerParameters<T> with_p(const T& p) const {
    MobilPlannerParameters<T> result(*this);
    result.set_p(p);
    return result;
  }
  /// acceleration threshold for changing lanes (Delta_a_th)
  /// @note @c threshold is expressed in units of m/s^2.
  /// @note @c threshold has a limited domain of [0.0, +Inf].
  const T& threshold() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kThreshold);
  }
  /// Setter that matches threshold().
  void set_threshold(const T& threshold) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kThreshold, threshold);
  }
  /// Fluent setter that matches threshold().
  /// Returns a copy of `this` with threshold set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MobilPlannerParameters<T> with_threshold(const T& threshold) const {
    MobilPlannerParameters<T> result(*this);
    result.set_threshold(threshold);
    return result;
  }
  /// maximum safe deceleration (b_safe)
  /// @note @c max_deceleration is expressed in units of m/s^2.
  /// @note @c max_deceleration has a limited domain of [0.0, +Inf].
  const T& max_deceleration() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMaxDeceleration);
  }
  /// Setter that matches max_deceleration().
  void set_max_deceleration(const T& max_deceleration) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMaxDeceleration, max_deceleration);
  }
  /// Fluent setter that matches max_deceleration().
  /// Returns a copy of `this` with max_deceleration set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MobilPlannerParameters<T> with_max_deceleration(const T& max_deceleration) const {
    MobilPlannerParameters<T> result(*this);
    result.set_max_deceleration(max_deceleration);
    return result;
  }
  //@}

  /// See MobilPlannerParametersIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return MobilPlannerParametersIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(p());
    result = result && (p() >= T(0.0));
    result = result && (p() <= T(1.0));
    result = result && !isnan(threshold());
    result = result && (threshold() >= T(0.0));
    result = result && !isnan(max_deceleration());
    result = result && (max_deceleration() >= T(0.0));
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower, Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 3, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 3, 1>::Constant(kInf);
    (*lower)(K::kP) = 0.0;
    (*upper)(K::kP) = 1.0;
    (*lower)(K::kThreshold) = 0.0;
    (*lower)(K::kMaxDeceleration) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The MobilPlannerParameters vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
