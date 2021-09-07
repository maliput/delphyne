// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/systems/primitives/linear_system.h>

namespace delphyne {

/// SimplePowertrain models a powertrain with first-order lag. It accepts
/// throttle as the input and outputs the applied lumped force from the vehicle
/// to the road.
///
/// Input:
///  - A unitless scalar value representing the throttle input to the power
///    system.
///
/// Output:
///  - The force transmitted from the vehicle to the road [N].
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class SimplePowertrain final : public drake::systems::LinearSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimplePowertrain);

  /// Constructs a simple powertrain model, specified via a fixed time-constant
  /// and scalar gain.  The inputs are as follows:
  /// @param time_constant is the rise time of the first-order lag [s].
  /// @param gain is the gain converting throttle input to force output [N].
  SimplePowertrain(double time_constant, double gain)
      : drake::systems::LinearSystem<T>(
            drake::systems::SystemTypeTag<SimplePowertrain>{}, drake::Vector1d(-1. / time_constant),
            drake::Vector1d(gain), drake::Vector1d(1. / time_constant), drake::Vector1d(0.), 0.0 /* time_period */),
        time_constant_(time_constant),
        gain_(gain) {}

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit SimplePowertrain(const SimplePowertrain<U>& other)
      : SimplePowertrain<T>(other.get_time_constant(), other.get_gain()) {}

  ~SimplePowertrain() override = default;

  const drake::systems::InputPort<T>& get_throttle_input_port() const {
    return drake::systems::System<T>::get_input_port(0);
  }

  const drake::systems::OutputPort<T>& get_force_output_port() const {
    return drake::systems::System<T>::get_output_port(0);
  }

  /// Accessors for the system constants.
  /// @{
  double get_time_constant() const { return time_constant_; }
  double get_gain() const { return gain_; }
  /// @}

 private:
  const double time_constant_{};
  const double gain_{};
};

}  // namespace delphyne
