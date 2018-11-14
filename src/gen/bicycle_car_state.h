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

/// Describes the row indices of a BicycleCarState.
struct BicycleCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kPsi = 0;
  static const int kPsiDot = 1;
  static const int kBeta = 2;
  static const int kVel = 3;
  static const int kSx = 4;
  static const int kSy = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BicycleCarStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BicycleCarState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BicycleCarStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c Psi defaults to 0.0 with unknown units.
  /// @arg @c Psi_dot defaults to 0.0 with unknown units.
  /// @arg @c beta defaults to 0.0 with unknown units.
  /// @arg @c vel defaults to 0.0 with unknown units.
  /// @arg @c sx defaults to 0.0 with unknown units.
  /// @arg @c sy defaults to 0.0 with unknown units.
  BicycleCarState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_Psi(0.0);
    this->set_Psi_dot(0.0);
    this->set_beta(0.0);
    this->set_vel(0.0);
    this->set_sx(0.0);
    this->set_sy(0.0);
  }

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_Psi(drake::symbolic::Variable("Psi"));
    this->set_Psi_dot(drake::symbolic::Variable("Psi_dot"));
    this->set_beta(drake::symbolic::Variable("beta"));
    this->set_vel(drake::symbolic::Variable("vel"));
    this->set_sx(drake::symbolic::Variable("sx"));
    this->set_sy(drake::symbolic::Variable("sy"));
  }

  BicycleCarState<T>* DoClone() const final { return new BicycleCarState; }

  /// @name Getters and Setters
  //@{
  /// yaw angle
  const T& Psi() const { return this->GetAtIndex(K::kPsi); }
  void set_Psi(const T& Psi) { this->SetAtIndex(K::kPsi, Psi); }
  /// yaw angular rate
  const T& Psi_dot() const { return this->GetAtIndex(K::kPsiDot); }
  void set_Psi_dot(const T& Psi_dot) { this->SetAtIndex(K::kPsiDot, Psi_dot); }
  /// slip angle at the center of mass
  const T& beta() const { return this->GetAtIndex(K::kBeta); }
  void set_beta(const T& beta) { this->SetAtIndex(K::kBeta, beta); }
  /// velocity magnitude
  const T& vel() const { return this->GetAtIndex(K::kVel); }
  void set_vel(const T& vel) { this->SetAtIndex(K::kVel, vel); }
  /// x-position at the center of mass
  const T& sx() const { return this->GetAtIndex(K::kSx); }
  void set_sx(const T& sx) { this->SetAtIndex(K::kSx, sx); }
  /// y-position at the center of mass
  const T& sy() const { return this->GetAtIndex(K::kSy); }
  void set_sy(const T& sy) { this->SetAtIndex(K::kSy, sy); }
  //@}

  /// See BicycleCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BicycleCarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(Psi());
    result = result && !isnan(Psi_dot());
    result = result && !isnan(beta());
    result = result && !isnan(vel());
    result = result && !isnan(sx());
    result = result && !isnan(sy());
    return result;
  }
};

}  // namespace delphyne
