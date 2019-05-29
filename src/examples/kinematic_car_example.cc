// Copyright 2018 Toyota Research Institute
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "examples/kinematic_car_model.h"
#include "examples/kinematic_car_state.h"

#include "malidrive/backend/driving_command.h"

namespace delphyne {
namespace {

const double kDuration{100.};                       // [s]
const double kTimeStep{.1};                         // [s]
const double kDrivingAcceleration{1.};              // [m/s^2]
const double kMaximumSteering{10. / 180. * M_PI};   // [rad]
const double kSteeringFrequency{0.1};               // [Hz]

// Logs `state` at `t` time of a kinematic car.
void LogState(const delphyne::KinematicCarState& state, double t) {
  std::cout << "--------------------------------------------" << std::endl;
  std::cout << "Time: " << t << std::endl;
  std::cout << "Inertial position: ("
            << state.inertial_position[0] << ", "
            << state.inertial_position[1] << ", "
            << state.inertial_position[2] << ")"
            << std::endl;
  std::cout << "Inertial orientation: ("
            << state.inertial_rotation.roll_angle() << ", "
            << state.inertial_rotation.pitch_angle() << ", "
            << state.inertial_rotation.yaw_angle() << ")"
            << std::endl;
  std::cout << "Velocity: " << state.velocity << std::endl;
  std::cout << "--------------------------------------------" << std::endl;
}

// Swings the steering command to make a car oscillate with a sine function of
// `t` time.
double SwingSteeringCommand(double t) {
  return kMaximumSteering * std::sin(2 * M_PI * kSteeringFrequency * t);
}

}  // namespace


int DoMain(int, char**) {
  auto kinematic_model = std::make_unique<delphyne::KinematicCarModel>(
      delphyne::KinematicCarModel::CarProperties(),
      delphyne::KinematicCarState());

  backend::DrivingCommand driving_command;
  driving_command.acceleration =  kDrivingAcceleration;

  double t{0.};
  while (t < kDuration) {
    // Sets the steering command.
    driving_command.steering = SwingSteeringCommand(t);
    kinematic_model->set_driving_command(driving_command);
    // Step the controller to move forward.
    kinematic_model->StepBy(kTimeStep);
    LogState(kinematic_model->get_state(), t);

    t += kTimeStep;
    t = std::min(t, kDuration);
  }

  return 0;
}


}  // namespace delphyne


int main(int argc, char** argv) { return delphyne::DoMain(argc, argv); }
