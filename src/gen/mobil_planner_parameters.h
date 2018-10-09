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
  MobilPlannerParameters()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_p(0.5);
    this->set_threshold(0.1);
    this->set_max_deceleration(4.0);
  }

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_p(drake::symbolic::Variable("p"));
    this->set_threshold(drake::symbolic::Variable("threshold"));
    this->set_max_deceleration(drake::symbolic::Variable("max_deceleration"));
  }

  MobilPlannerParameters<T>* DoClone() const final {
    return new MobilPlannerParameters;
  }

  /// @name Getters and Setters
  //@{
  /// politeness factor (0.0 is purely egoistic, higher values increase
  /// politeness)
  /// @note @c p is expressed in units of dimensionless.
  /// @note @c p has a limited domain of [0.0, 1.0].
  const T& p() const { return this->GetAtIndex(K::kP); }
  void set_p(const T& p) { this->SetAtIndex(K::kP, p); }
  /// acceleration threshold for changing lanes (Delta_a_th)
  /// @note @c threshold is expressed in units of m/s^2.
  /// @note @c threshold has a limited domain of [0.0, +Inf].
  const T& threshold() const { return this->GetAtIndex(K::kThreshold); }
  void set_threshold(const T& threshold) {
    this->SetAtIndex(K::kThreshold, threshold);
  }
  /// maximum safe deceleration (b_safe)
  /// @note @c max_deceleration is expressed in units of m/s^2.
  /// @note @c max_deceleration has a limited domain of [0.0, +Inf].
  const T& max_deceleration() const {
    return this->GetAtIndex(K::kMaxDeceleration);
  }
  void set_max_deceleration(const T& max_deceleration) {
    this->SetAtIndex(K::kMaxDeceleration, max_deceleration);
  }
  //@}

  /// See MobilPlannerParametersIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return MobilPlannerParametersIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(p());
    result = result && (p() >= T(0.0));
    result = result && (p() <= T(1.0));
    result = result && !isnan(threshold());
    result = result && (threshold() >= T(0.0));
    result = result && !isnan(max_deceleration());
    result = result && (max_deceleration() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(4);
    (*value)[0] = p() - T(0.0);
    (*value)[1] = T(1.0) - p();
    (*value)[2] = threshold() - T(0.0);
    (*value)[3] = max_deceleration() - T(0.0);
  }
};

}  // namespace delphyne
