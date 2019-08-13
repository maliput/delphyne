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
  DynamicBicycleCarParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
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

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  DynamicBicycleCarParams(const DynamicBicycleCarParams& other) : drake::systems::BasicVector<T>(other.values()) {}
  DynamicBicycleCarParams(DynamicBicycleCarParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  DynamicBicycleCarParams& operator=(const DynamicBicycleCarParams& other) {
    this->values() = other.values();
    return *this;
  }
  DynamicBicycleCarParams& operator=(DynamicBicycleCarParams&& other) noexcept {
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

  DynamicBicycleCarParams<T>* DoClone() const final { return new DynamicBicycleCarParams; }

  /// @name Getters and Setters
  //@{
  /// Mass of X1 vehicle.
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMass);
  }
  /// Setter that matches mass().
  void set_mass(const T& mass) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMass, mass);
  }
  /// Fluent setter that matches mass().
  /// Returns a copy of `this` with mass set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_mass(const T& mass) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_mass(mass);
    return result;
  }
  /// moment of inertia.
  /// @note @c izz is expressed in units of kgm^2.
  /// @note @c izz has a limited domain of [0.0, +Inf].
  const T& izz() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kIzz);
  }
  /// Setter that matches izz().
  void set_izz(const T& izz) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kIzz, izz);
  }
  /// Fluent setter that matches izz().
  /// Returns a copy of `this` with izz set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_izz(const T& izz) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_izz(izz);
    return result;
  }
  /// Front cornering stiffness.
  /// @note @c c_alpha_f is expressed in units of N/rad.
  /// @note @c c_alpha_f has a limited domain of [0.0, +Inf].
  const T& c_alpha_f() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kCAlphaF);
  }
  /// Setter that matches c_alpha_f().
  void set_c_alpha_f(const T& c_alpha_f) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kCAlphaF, c_alpha_f);
  }
  /// Fluent setter that matches c_alpha_f().
  /// Returns a copy of `this` with c_alpha_f set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_c_alpha_f(const T& c_alpha_f) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_c_alpha_f(c_alpha_f);
    return result;
  }
  /// Rear cornering stiffness.
  /// @note @c c_alpha_r is expressed in units of N/rad.
  /// @note @c c_alpha_r has a limited domain of [0.0, +Inf].
  const T& c_alpha_r() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kCAlphaR);
  }
  /// Setter that matches c_alpha_r().
  void set_c_alpha_r(const T& c_alpha_r) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kCAlphaR, c_alpha_r);
  }
  /// Fluent setter that matches c_alpha_r().
  /// Returns a copy of `this` with c_alpha_r set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_c_alpha_r(const T& c_alpha_r) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_c_alpha_r(c_alpha_r);
    return result;
  }
  /// Coefficient of friction between tire and road surface.
  /// @note @c mu is expressed in units of dimensionless.
  /// @note @c mu has a limited domain of [0.0, +Inf].
  const T& mu() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMu);
  }
  /// Setter that matches mu().
  void set_mu(const T& mu) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMu, mu);
  }
  /// Fluent setter that matches mu().
  /// Returns a copy of `this` with mu set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_mu(const T& mu) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_mu(mu);
    return result;
  }
  /// Distance from control point to front axle (referred to as 'a' in Bobier).
  /// @note @c Lf is expressed in units of m.
  /// @note @c Lf has a limited domain of [0.0, +Inf].
  const T& Lf() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLf);
  }
  /// Setter that matches Lf().
  void set_Lf(const T& Lf) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLf, Lf);
  }
  /// Fluent setter that matches Lf().
  /// Returns a copy of `this` with Lf set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_Lf(const T& Lf) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_Lf(Lf);
    return result;
  }
  /// Distance from rear axle to control point (referred to as 'b' in Bobier).
  /// @note @c Lb is expressed in units of m.
  /// @note @c Lb has a limited domain of [0.0, +Inf].
  const T& Lb() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLb);
  }
  /// Setter that matches Lb().
  void set_Lb(const T& Lb) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLb, Lb);
  }
  /// Fluent setter that matches Lb().
  /// Returns a copy of `this` with Lb set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_Lb(const T& Lb) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_Lb(Lb);
    return result;
  }
  /// Height of vehicle's control point Cp.
  /// @note @c p_LoCp_z is expressed in units of m.
  /// @note @c p_LoCp_z has a limited domain of [0.0, +Inf].
  const T& p_LoCp_z() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPLocpZ);
  }
  /// Setter that matches p_LoCp_z().
  void set_p_LoCp_z(const T& p_LoCp_z) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPLocpZ, p_LoCp_z);
  }
  /// Fluent setter that matches p_LoCp_z().
  /// Returns a copy of `this` with p_LoCp_z set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_p_LoCp_z(const T& p_LoCp_z) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_p_LoCp_z(p_LoCp_z);
    return result;
  }
  /// An approximate value for gravitational acceleration.
  /// @note @c gravity is expressed in units of m/s^2.
  /// @note @c gravity has a limited domain of [0.0, +Inf].
  const T& gravity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kGravity);
  }
  /// Setter that matches gravity().
  void set_gravity(const T& gravity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kGravity, gravity);
  }
  /// Fluent setter that matches gravity().
  /// Returns a copy of `this` with gravity set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarParams<T> with_gravity(const T& gravity) const {
    DynamicBicycleCarParams<T> result(*this);
    result.set_gravity(gravity);
    return result;
  }
  //@}

  /// See DynamicBicycleCarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DynamicBicycleCarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
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

  void GetElementBounds(Eigen::VectorXd* lower, Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 9, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 9, 1>::Constant(kInf);
    (*lower)(K::kMass) = 0.0;
    (*lower)(K::kIzz) = 0.0;
    (*lower)(K::kCAlphaF) = 0.0;
    (*lower)(K::kCAlphaR) = 0.0;
    (*lower)(K::kMu) = 0.0;
    (*lower)(K::kLf) = 0.0;
    (*lower)(K::kLb) = 0.0;
    (*lower)(K::kPLocpZ) = 0.0;
    (*lower)(K::kGravity) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The DynamicBicycleCarParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
