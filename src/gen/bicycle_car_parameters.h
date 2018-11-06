// Copyright 2018 Toyota Research Institute

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <drake/common/drake_bool.h>
#include <drake/common/dummy_value.h>
#include <drake/common/never_destroyed.h>
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>

namespace delphyne {

/// Describes the row indices of a BicycleCarParameters.
struct BicycleCarParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kLf = 1;
  static const int kLr = 2;
  static const int kIz = 3;
  static const int kCf = 4;
  static const int kCr = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BicycleCarParametersIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BicycleCarParameters final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BicycleCarParametersIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 2278.0 kg.
  /// @arg @c lf defaults to 1.292 m.
  /// @arg @c lr defaults to 1.515 m.
  /// @arg @c Iz defaults to 3210.0 kg m^2.
  /// @arg @c Cf defaults to 10.8e4 N / rad.
  /// @arg @c Cr defaults to 10.8e4 N / rad.
  BicycleCarParameters() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(2278.0);
    this->set_lf(1.292);
    this->set_lr(1.515);
    this->set_Iz(3210.0);
    this->set_Cf(10.8e4);
    this->set_Cr(10.8e4);
  }

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_mass(drake::symbolic::Variable("mass"));
    this->set_lf(drake::symbolic::Variable("lf"));
    this->set_lr(drake::symbolic::Variable("lr"));
    this->set_Iz(drake::symbolic::Variable("Iz"));
    this->set_Cf(drake::symbolic::Variable("Cf"));
    this->set_Cr(drake::symbolic::Variable("Cr"));
  }

  BicycleCarParameters<T>* DoClone() const final {
    return new BicycleCarParameters;
  }

  /// @name Getters and Setters
  //@{
  /// mass
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const { return this->GetAtIndex(K::kMass); }
  void set_mass(const T& mass) { this->SetAtIndex(K::kMass, mass); }
  /// distance from the center of mass to the front axle
  /// @note @c lf is expressed in units of m.
  /// @note @c lf has a limited domain of [0.0, +Inf].
  const T& lf() const { return this->GetAtIndex(K::kLf); }
  void set_lf(const T& lf) { this->SetAtIndex(K::kLf, lf); }
  /// distance from the center of mass to the rear axle
  /// @note @c lr is expressed in units of m.
  /// @note @c lr has a limited domain of [0.0, +Inf].
  const T& lr() const { return this->GetAtIndex(K::kLr); }
  void set_lr(const T& lr) { this->SetAtIndex(K::kLr, lr); }
  /// moment of inertia about the yaw-axis
  /// @note @c Iz is expressed in units of kg m^2.
  /// @note @c Iz has a limited domain of [0.0, +Inf].
  const T& Iz() const { return this->GetAtIndex(K::kIz); }
  void set_Iz(const T& Iz) { this->SetAtIndex(K::kIz, Iz); }
  /// cornering stiffness (front)
  /// @note @c Cf is expressed in units of N / rad.
  /// @note @c Cf has a limited domain of [0.0, +Inf].
  const T& Cf() const { return this->GetAtIndex(K::kCf); }
  void set_Cf(const T& Cf) { this->SetAtIndex(K::kCf, Cf); }
  /// cornering stiffness (rear)
  /// @note @c Cr is expressed in units of N / rad.
  /// @note @c Cr has a limited domain of [0.0, +Inf].
  const T& Cr() const { return this->GetAtIndex(K::kCr); }
  void set_Cr(const T& Cr) { this->SetAtIndex(K::kCr, Cr); }
  //@}

  /// See BicycleCarParametersIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BicycleCarParametersIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.0));
    result = result && !isnan(lf());
    result = result && (lf() >= T(0.0));
    result = result && !isnan(lr());
    result = result && (lr() >= T(0.0));
    result = result && !isnan(Iz());
    result = result && (Iz() >= T(0.0));
    result = result && !isnan(Cf());
    result = result && (Cf() >= T(0.0));
    result = result && !isnan(Cr());
    result = result && (Cr() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(6);
    (*value)[0] = mass() - T(0.0);
    (*value)[1] = lf() - T(0.0);
    (*value)[2] = lr() - T(0.0);
    (*value)[3] = Iz() - T(0.0);
    (*value)[4] = Cf() - T(0.0);
    (*value)[5] = Cr() - T(0.0);
  }
};

}  // namespace delphyne
