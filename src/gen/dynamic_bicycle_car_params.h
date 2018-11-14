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

/// Describes the row indices of a DynamicBicycleCarParams.
struct DynamicBicycleCarParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 9;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kIzz = 1;
  static const int kCAlphaF = 2;
  static const int kCAlphaR = 3;
  static const int kMu = 4;
  static const int kLf = 5;
  static const int kLb = 6;
  static const int kPLocpZ = 7;
  static const int kGravity = 8;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `DynamicBicycleCarParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class DynamicBicycleCarParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef DynamicBicycleCarParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 1823.0 kg.
  /// @arg @c izz defaults to 2000.0 kgm^2.
  /// @arg @c c_alpha_f defaults to 115000 N/rad.
  /// @arg @c c_alpha_r defaults to 155000 N/rad.
  /// @arg @c mu defaults to 0.55 dimensionless.
  /// @arg @c Lf defaults to 1.54 m.
  /// @arg @c Lb defaults to 1.21 m.
  /// @arg @c p_LoCp_z defaults to 0.508 m.
  /// @arg @c gravity defaults to 9.81 m/s^2.
  DynamicBicycleCarParams()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(1823.0);
    this->set_izz(2000.0);
    this->set_c_alpha_f(115000);
    this->set_c_alpha_r(155000);
    this->set_mu(0.55);
    this->set_Lf(1.54);
    this->set_Lb(1.21);
    this->set_p_LoCp_z(0.508);
    this->set_gravity(9.81);
  }

  /// Create a drake::symbolic::Variable for each element with the known
  /// variable
  /// name.  This is only available for T == drake::symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_mass(drake::symbolic::Variable("mass"));
    this->set_izz(drake::symbolic::Variable("izz"));
    this->set_c_alpha_f(drake::symbolic::Variable("c_alpha_f"));
    this->set_c_alpha_r(drake::symbolic::Variable("c_alpha_r"));
    this->set_mu(drake::symbolic::Variable("mu"));
    this->set_Lf(drake::symbolic::Variable("Lf"));
    this->set_Lb(drake::symbolic::Variable("Lb"));
    this->set_p_LoCp_z(drake::symbolic::Variable("p_LoCp_z"));
    this->set_gravity(drake::symbolic::Variable("gravity"));
  }

  DynamicBicycleCarParams<T>* DoClone() const final {
    return new DynamicBicycleCarParams;
  }

  /// @name Getters and Setters
  //@{
  /// Mass of X1 vehicle.
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const { return this->GetAtIndex(K::kMass); }
  void set_mass(const T& mass) { this->SetAtIndex(K::kMass, mass); }
  /// moment of inertia.
  /// @note @c izz is expressed in units of kgm^2.
  /// @note @c izz has a limited domain of [0.0, +Inf].
  const T& izz() const { return this->GetAtIndex(K::kIzz); }
  void set_izz(const T& izz) { this->SetAtIndex(K::kIzz, izz); }
  /// Front cornering stiffness.
  /// @note @c c_alpha_f is expressed in units of N/rad.
  /// @note @c c_alpha_f has a limited domain of [0.0, +Inf].
  const T& c_alpha_f() const { return this->GetAtIndex(K::kCAlphaF); }
  void set_c_alpha_f(const T& c_alpha_f) {
    this->SetAtIndex(K::kCAlphaF, c_alpha_f);
  }
  /// Rear cornering stiffness.
  /// @note @c c_alpha_r is expressed in units of N/rad.
  /// @note @c c_alpha_r has a limited domain of [0.0, +Inf].
  const T& c_alpha_r() const { return this->GetAtIndex(K::kCAlphaR); }
  void set_c_alpha_r(const T& c_alpha_r) {
    this->SetAtIndex(K::kCAlphaR, c_alpha_r);
  }
  /// Coefficient of friction between tire and road surface.
  /// @note @c mu is expressed in units of dimensionless.
  /// @note @c mu has a limited domain of [0.0, +Inf].
  const T& mu() const { return this->GetAtIndex(K::kMu); }
  void set_mu(const T& mu) { this->SetAtIndex(K::kMu, mu); }
  /// Distance from control point to front axle (referred to as 'a' in Bobier).
  /// @note @c Lf is expressed in units of m.
  /// @note @c Lf has a limited domain of [0.0, +Inf].
  const T& Lf() const { return this->GetAtIndex(K::kLf); }
  void set_Lf(const T& Lf) { this->SetAtIndex(K::kLf, Lf); }
  /// Distance from rear axle to control point (referred to as 'b' in Bobier).
  /// @note @c Lb is expressed in units of m.
  /// @note @c Lb has a limited domain of [0.0, +Inf].
  const T& Lb() const { return this->GetAtIndex(K::kLb); }
  void set_Lb(const T& Lb) { this->SetAtIndex(K::kLb, Lb); }
  /// Height of vehicle's control point Cp.
  /// @note @c p_LoCp_z is expressed in units of m.
  /// @note @c p_LoCp_z has a limited domain of [0.0, +Inf].
  const T& p_LoCp_z() const { return this->GetAtIndex(K::kPLocpZ); }
  void set_p_LoCp_z(const T& p_LoCp_z) {
    this->SetAtIndex(K::kPLocpZ, p_LoCp_z);
  }
  /// An approximate value for gravitational acceleration.
  /// @note @c gravity is expressed in units of m/s^2.
  /// @note @c gravity has a limited domain of [0.0, +Inf].
  const T& gravity() const { return this->GetAtIndex(K::kGravity); }
  void set_gravity(const T& gravity) { this->SetAtIndex(K::kGravity, gravity); }
  //@}

  /// See DynamicBicycleCarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DynamicBicycleCarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.0));
    result = result && !isnan(izz());
    result = result && (izz() >= T(0.0));
    result = result && !isnan(c_alpha_f());
    result = result && (c_alpha_f() >= T(0.0));
    result = result && !isnan(c_alpha_r());
    result = result && (c_alpha_r() >= T(0.0));
    result = result && !isnan(mu());
    result = result && (mu() >= T(0.0));
    result = result && !isnan(Lf());
    result = result && (Lf() >= T(0.0));
    result = result && !isnan(Lb());
    result = result && (Lb() >= T(0.0));
    result = result && !isnan(p_LoCp_z());
    result = result && (p_LoCp_z() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(9);
    (*value)[0] = mass() - T(0.0);
    (*value)[1] = izz() - T(0.0);
    (*value)[2] = c_alpha_f() - T(0.0);
    (*value)[3] = c_alpha_r() - T(0.0);
    (*value)[4] = mu() - T(0.0);
    (*value)[5] = Lf() - T(0.0);
    (*value)[6] = Lb() - T(0.0);
    (*value)[7] = p_LoCp_z() - T(0.0);
    (*value)[8] = gravity() - T(0.0);
  }
};

}  // namespace delphyne
