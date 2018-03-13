// Copyright 2017 Toyota Research Institute

#include <cstdlib>
#include <memory>
#include <string>

#include "backend/simulation_runner.h"

#include <drake/automotive/automotive_simulator.h>
#include <drake/common/find_resource.h>

namespace delphyne {
namespace backend {
namespace {

// Generates a channel name based on a given string.
std::string MakeChannelName(const std::string& name) {
  const std::string default_prefix{"DRIVING_COMMAND"};
  if (name.empty()) {
    return default_prefix;
  }
  return default_prefix + "_" + name;
}

drake::automotive::Curve2<double> MakeCurve(double radius, double inset) {
  // TODO(jwnimmer-tri) This function will be rewritten once we have
  // proper splines.  Don't try too hard to understand it.  Run the
  // demo to see it first, and only then try to understand the code.

  typedef drake::automotive::Curve2<double>::Point2 Point2d;
  std::vector<Point2d> waypoints;

  // Start (0, +i).
  // Straight right to (+r, +i).
  // Loop around (+i, +r).
  // Straight back to (+i, 0).
  waypoints.push_back({0.0, inset});
  for (int theta_deg = -90; theta_deg <= 180; ++theta_deg) {
    const Point2d center{radius, radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius - inset)));
  }
  waypoints.push_back({inset, 0.0});

  // Start (+i, 0).
  // Straight down to (+i, -r).
  // Loop around (-r, +i).
  // Straight back to start (implicitly via segment to waypoints[0]).
  for (int theta_deg = 0; theta_deg >= -270; --theta_deg) {
    const Point2d center{-radius, -radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius + inset)));
  }

  // Many copies.
  const int kNumCopies = 100;
  std::vector<Point2d> looped_waypoints;
  for (int copies = 0; copies < kNumCopies; ++copies) {
    std::copy(waypoints.begin(), waypoints.end(),
              std::back_inserter(looped_waypoints));
  }
  looped_waypoints.push_back(waypoints.front());

  return drake::automotive::Curve2<double>(looped_waypoints);
}

std::tuple<drake::automotive::Curve2<double>, double, double>
CreateTrajectoryParams(int index) {
  // The possible curves to trace (lanes).
  static const std::vector<drake::automotive::Curve2<double>> curves{
      MakeCurve(40.0, 0.0),  // BR
      MakeCurve(40.0, 4.0),  // BR
      MakeCurve(40.0, 8.0),
  };

  // Magic car placement to make a good visual demo.
  const auto& curve = curves[index % curves.size()];
  const double start_time = (index / curves.size()) * 0.8;
  const double kSpeed = 2.0;
  return std::make_tuple(curve, kSpeed, start_time);
}

int main(int argc, char* argv[]) {
  // Instantiates a simulator.
  auto simulator =
      std::make_unique<delphyne::backend::AutomotiveSimulator<double>>();

  // Adds a Prius car.
  drake::automotive::SimpleCarState<double> state;
  state.set_y(0.0);
  simulator->AddPriusSimpleCar("0", MakeChannelName("0"), state);

  // Adds a Loadable Prius Simple car.
  drake::automotive::SimpleCarState<double> state2;
  state2.set_y(4.0);
  std::map<std::string, linb::any> simple_params;
  if (simulator->AddLoadableCar("LoadablePriusSimpleCar", simple_params, "1",
                                &state2) < 0) {
    return 1;
  }

  // Add a Loadable Prius Trajectory Car.
  const auto& params = CreateTrajectoryParams(0);
  std::map<std::string, linb::any> traj_params;
  traj_params["curve"] = std::get<0>(params);
  drake::automotive::TrajectoryCarState<double> state3;
  state3.set_speed(std::get<1>(params));
  state3.set_position(std::get<2>(params));
  if (simulator->AddLoadableCar("LoadablePriusTrajectoryCar", traj_params,
                                "TrajectoryCar0", &state3) < 0) {
    return 1;
  }

  // Instantiates the simulator runner and starts it.
  const double kTimeStep = 0.001;
  delphyne::backend::SimulatorRunner prius_sim_runner(std::move(simulator),
                                                      kTimeStep);
  prius_sim_runner.Start();

  // Zzzzzz.
  delphyne::backend::WaitForShutdown();
  return 0;
}

}  // namespace
}  // namespace backend
}  // namespace delphyne

int main(int argc, char* argv[]) { return delphyne::backend::main(argc, argv); }
