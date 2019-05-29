// Copyright 2018 Toyota Research Institute
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "examples/pure_pursuit.h"
#include "examples/kinematic_car_model.h"
#include "examples/kinematic_car_state.h"
#include "examples/opendriver_controller.h"
#include "examples/opendriver_controller_state.h"

#include "malidrive/backend/driving_command.h"
#include "malidrive/backend/step_strategy.h"
#include "malidrive/utility/resources.h"


namespace delphyne {
namespace {

// Available demos.
enum class DemoType {
  kLineSingleLane = 1,
  kArcSingleLane,
  kRoundabout,
  kLShapeSection,
  kLShapeRoad,
  kTShapeRoad,
  kCrossing8Course,
  kInvalid,
};

// Information needed for each demo file.
struct DemoInformation {
  std::string file_path;
  std::string road_description;
  int track_id;
  int lane_id;
  double initial_s;
};

// Demo constants
const char kDemoUsage[] =
  "Usage:\n\r"
  "\topendrive_demo --demo=X\n\r"
  "Where X could be:\n\r"
  "\t'1': line lane\n\r"
  "\t'2': arc lane\n\r"
  "\t'3': roundabout lane\n\r"
  "\t'4': L shape with LaneSections\n\r"
  "\t'5': L shape with roads\n\r"
  "\t'6': T shape with roads\n\r"
  "\t'7': 8 shape crossing\n\r"
  "Any other option will be consider wrong and would cause the program to end.";

const std::map<DemoType, DemoInformation> kDemoInformation{
  {DemoType::kLineSingleLane,
   {"odr/SingleLane.xodr", "Single line lane of 100m length", 1, 1, 0.}},
  {DemoType::kArcSingleLane,
   {"odr/ArcLane.xodr", "Single arc lane of 100m length and 40m of radius", 1,
   1, 0.}},
  {DemoType::kRoundabout,
   {"odr/Roundabout.xodr",
    "Single lane roundabout of 200m length and ~31.83m of radius", 1, 1, 0.}},
  {DemoType::kLShapeSection,
   {"odr/LShapeSection.xodr",
    "Single road with 3 lane sections with line-arc-line geometry.", 1, 1, 0.}},
  {DemoType::kLShapeRoad,
   {"odr/LShapeRoad.xodr",
    "3 roads connected each with line, arc and line geometry respectively.", 1,
    1, 0.}},
  {DemoType::kTShapeRoad,
   {"odr/TShapeRoad.xodr", "T intersection road with double hand roads", 1, 1,
   0.}},
  {DemoType::kCrossing8Course,
   {"odr/Crossing8Course.xodr", "Crossing with 8 shape.", 500, -1, 0.}},
};

const char kCSVFilePath[] = "./inertial_log.csv";
const double kDuration{100.};                         //  [s]
const double kTimeStep{.1};                           //  [s]
const double kSLookahead{20.};                        //  [m]
const double kDrivingAcceleration{.4};                //  [m/s^2]
const double kLinearTolerance{2e-3};                  //  [m]

// Prints demo usage.
void PrintUsage() { std::cout << kDemoUsage << std::endl; }

// Prints `demo` information.
void PrintDemoInformation(DemoType demo) {
  DRAKE_DEMAND(demo != DemoType::kInvalid);
  std::cout << kDemoInformation.at(demo).road_description << std::endl;
}

// Returns the DemoType from the CLI arguments (`argc` and `argv`) that the
// executable receives.
DemoType ParseDemoType(int argc, char* argv[]) {
  DRAKE_DEMAND(argc > 0);
  DRAKE_DEMAND(argv != nullptr);
  static const std::string kDemoPrefix{"--demo="};

  if (argc == 2) {
    const std::string arguments(argv[1]);
    auto it = arguments.find(kDemoPrefix);
    if (it == 0) {
      const int demo_index = std::stoi(arguments.substr(kDemoPrefix.length()));
      return demo_index >= static_cast<int>(DemoType::kLineSingleLane) &&
             demo_index < static_cast<int>(DemoType::kInvalid)
                ? static_cast<DemoType>(demo_index) : DemoType::kInvalid;
    }
  }
  return DemoType::kInvalid;
}

// Logs `state` at `t` time of a kinematic car.
void LogState(const KinematicCarState& state,
              const OpenDriverControllerState& controller_state,
              const backend::DrivingCommand& driving_command,
              double t) {
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
  std::cout << "Track: ";
  controller_state.goal.lane_coord.print();
  std::cout << "Steering: " << driving_command.steering << std::endl;
  std::cout << "Acceleration: " << driving_command.acceleration << std::endl;

  std::cout << "--------------------------------------------" << std::endl;
}

// Logs car and target inertial positions to `os` stream.
void LogCSV(
    std::ofstream& os, double car_x, double car_y, double target_x,
    double target_y) {
  os << car_x << "," << car_y << "," << target_x << "," << target_y
      << std::endl;
}


// Returns an OpenDRIVE manager that loaded `file_path` xodr map.
std::unique_ptr<OpenDrive::OdrManager> LoadOpenDriveManagerFor(
    const std::string& file_path) {
  auto manager = std::make_unique<OpenDrive::OdrManager>();
  if (!manager->loadFile(file_path)) {
     throw std::runtime_error("File could not be loaded.");
  }
  manager->activatePosition(manager->createPosition());
  return std::move(manager);
}

std::unique_ptr<OpenDriverController> MakeOpenDriverController(
    const OpenDrive::LaneCoord& target_lane_goal,
    const KinematicCarModel::CarProperties& car_properties,
    OpenDrive::OdrManager* manager) {
  auto strategy = backend::StepStrategyFactory().Make(
      backend::StepStrategyFactory::Strategy::kPickFirstLane, manager,
      kLinearTolerance);
  return std::make_unique<OpenDriverController>(
      OpenDriverControllerParams(
          kSLookahead, kDrivingAcceleration, car_properties),
      OpenDriverControllerState(backend::PurePursuitOpenDriveGoal(
          target_lane_goal, backend::LaneDirection::kWithS,
          backend::GoalType::kRoadpoint)),
      std::move(strategy),
      manager);
}

drake::Vector3<double> XODRLaneCoordinateToInertialPosition(
  const OpenDrive::LaneCoord& lane_coord, OpenDrive::OdrManager* manager) {
  manager->setLanePos(lane_coord);
  DRAKE_DEMAND(manager->lane2inertial());
  const OpenDrive::Coord inertial_coord = manager->getInertialPos();
  return {inertial_coord.getX(), inertial_coord.getY(), inertial_coord.getZ()};
}

}  // namespace


int DoMain(int argc, char** argv) {
  const DemoType demo = ParseDemoType(argc, argv);
  if (demo == DemoType::kInvalid) {
    PrintUsage();
    return 0;
  }
  PrintDemoInformation(demo);

  auto manager = LoadOpenDriveManagerFor(
      utility::FindResource(kDemoInformation.at(demo).file_path));

  const OpenDrive::LaneCoord start_lane_coord(
      kDemoInformation.at(demo).track_id, kDemoInformation.at(demo).lane_id,
      kDemoInformation.at(demo).initial_s);

  const drake::Vector3<double> initial_inertial_position =
      XODRLaneCoordinateToInertialPosition(start_lane_coord, manager.get());
  auto kinematic_model = std::make_unique<KinematicCarModel>(
      KinematicCarModel::CarProperties(),
      KinematicCarState(initial_inertial_position,
                                 drake::math::RollPitchYaw<double>(0., 0., 0.),
                                 0.));

  auto opendrive_controller = MakeOpenDriverController(
      start_lane_coord, kinematic_model->get_car_properties(), manager.get());

  double t{0.};

  std::ofstream os;
  os.open(kCSVFilePath);

  while (t < kDuration) {
    // Computes the driving command.
    const backend::DrivingCommand driving_command =
        opendrive_controller->Drive(kinematic_model->get_state());
    kinematic_model->set_driving_command(driving_command);

    // Step the controller to move forward.
    kinematic_model->StepBy(kTimeStep);

    LogState(
        kinematic_model->get_state(), opendrive_controller->get_state(),
        driving_command, t);

    manager->setLanePos(opendrive_controller->get_state().goal.lane_coord);
    manager->lane2inertial();
    const OpenDrive::Coord inertial_target_goal = manager->getInertialPos();
    LogCSV(os, kinematic_model->get_state().inertial_position[0],
           kinematic_model->get_state().inertial_position[1],
           inertial_target_goal.getX(), inertial_target_goal.getY());

    t += kTimeStep;
    t = std::min(t, kDuration);
  }
  os.close();

  return 0;
}

}  // namespace delphyne


int main(int argc, char** argv) { return delphyne::DoMain(argc, argv); }
