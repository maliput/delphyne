// Copyright 2019 Toyota Research Institute

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <opendrive/OdrManager.hh>
#include <opendrive/OdrCoord.hh>
#include <opendrive/OdrTrackCoord.hh>
#include <opendrive/OdrLaneCoord.hh>

#include "gen/rail_follower_params.h"
#include "gen/rail_follower_state.h"

#include "drake/automotive/automotive_simulator.h"

#include "drake/common/drake_assert.h"
#include "drake/lcm/drake_lcm.h"

#include "examples/malidrive_automotive_simulator.h"

#include <malidrive/loader.h>
#include <malidrive/constants.h>
#include <malidrive/road_geometry_configuration.h>
#include <malidrive/road_network_configuration.h>
#include <malidrive/world_to_opendrive_transform.h>

#include "maliput/api/road_geometry.h"

#include <malidrive/utility/resources.h>

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
  kRRFigure8,
  kRRLongRoad,
  kInvalid,
};

// Information needed for each demo file.
struct DemoInformation {
  std::string file_path;
  std::string road_description;
  std::string lane_id;
  double s{};
  bool with_s{};
};

// Demo constants
const double kMaliputRailcarSpeed{25.0};  // [m/s]
const double kTargetRealTime{1.0};
const double kSimulationDuration{30.};    // [s]
const bool kAddLogger{true};

// Demo constants
const char kDemoUsage[] =
  "Usage:\n\r"
  "\tmalidrive_demo --demo=X\n\r"
  "Where X could be:\n\r"
  "\t'1': line lane\n\r"
  "\t'2': arc lane\n\r"
  "\t'3': roundabout lane\n\r"
  "\t'4': L shape with LaneSections\n\r"
  "\t'5': L shape with roads\n\r"
  "\t'6': T shape with roads\n\r"
  "\t'7': 8 shape crossing\n\r"
  "\t'9': 9 long road with turning lanes.\n\r"
  "Any other option will be consider wrong and would cause the program to end.";

const std::map<DemoType, DemoInformation> kDemoInformation{
  {DemoType::kLineSingleLane,
   {"odr/SingleLane.xodr", "Single line lane of 100m length", "1_0_-1", 0.,
    true}},
  {DemoType::kArcSingleLane,
   {"odr/ArcLane.xodr", "Single arc lane of 100m length and 40m of radius",
    "1_0_1", 0., true}},
  {DemoType::kRoundabout,
   {"odr/Roundabout.xodr",
    "Single lane roundabout of 200m length and ~31.83m of radius", "1_0_1",
    0., true}},
  {DemoType::kLShapeSection,
   {"odr/LShapeSection.xodr",
    "Single road with 3 lane sections with line-arc-line geometry.", "1_0_1",
    0., true}},
  {DemoType::kLShapeRoad,
   {"odr/LShapeRoad.xodr",
    "3 roads connected each with line, arc and line geometry respectively.",
    "1_0_1", 0., true}},
  {DemoType::kTShapeRoad,
   {"odr/TShapeRoad.xodr", "T intersection road with double hand roads",
    "1_0_1", 0., true}},
  {DemoType::kCrossing8Course,
   {"odr/Crossing8Course.xodr", "Crossing with 8 shape.", "514_0_-1", 0.,
    true}},
  {DemoType::kRRFigure8,
   {"odr/RRFigure8.xodr", "Crossing with 8 shape (another).", "4_0_-1", 80.,
    true}},
  {DemoType::kRRLongRoad,
   {"odr/RRLongRoad.xodr", "Long road with turning lanes.", "3_0_-6", 16.,
    false}},
};

// Returns the file name from `file_path` by splitting the string at the last
// position of '/' character till the end.
std::string GetFileNameFromPath(const std::string& file_path) {
  auto it = file_path.find_last_of('/');
  return it == std::string::npos ? file_path : file_path.substr(it + 1);
}

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
  static const std::string kDemoPreffix{"--demo="};

  if (argc == 2) {
    const std::string arguments(argv[1]);
    auto it = arguments.find(kDemoPreffix);
    if (it == 0) {
      const int demo_index = std::stoi(arguments.substr(kDemoPreffix.length()));
      return demo_index >= static_cast<int>(DemoType::kLineSingleLane) &&
             demo_index < static_cast<int>(DemoType::kInvalid)
                ? static_cast<DemoType>(demo_index) : DemoType::kInvalid;
    }
  }
  return DemoType::kInvalid;
}

// Builds and adds a malidrive::RoadGeometry to the `simulator`. RoadGeometry's
// ID will be `rg_id`.
// @throws When `simulator` is nullptr.
// @returns The pointer to the built RoadGeometry. Its lifetime will be managed
// by `simulator`.
const maliput::api::RoadNetwork*
AddMalidriveRoadNetwork(
    const std::string& xodr_file_path,
    const std::string& rg_id,
    MalidriveAutomotiveSimulator* simulator) {
  DRAKE_DEMAND(simulator != nullptr);

  return simulator->SetRoadNetwork(malidrive::Load(
    {
      {
        maliput::api::RoadGeometryId(
          GetFileNameFromPath(xodr_file_path)),
        xodr_file_path,
        malidrive::Constants::kLinearTolerance,
          malidrive::Constants::kAngularTolerance,
          malidrive::Constants::kScaleLength,
          malidrive::InertialToLaneMappingConfig(
              malidrive::Constants::kExplorationRadius,
              malidrive::Constants::kNumIterations)
      }
    },
    malidrive::WorldToOpenDriveTransform::Identity()));
}

// Adds a MaliputRailCar to `simulator` within `rg`.
//
// It will be placed in the lane whose ID is `lane_id` and at `start_s`
// distance from the origin of the lane. The car will point towards +s direction
// when `with_s` is true.
// @returns The car ID.
int AddMaliputRailCar(delphyne::MalidriveAutomotiveSimulator* simulator,
                      const maliput::api::RoadGeometry* rg,
                      const std::string& lane_id,
                      double start_s,
                      bool with_s) {
  DRAKE_DEMAND(simulator != nullptr);
  DRAKE_DEMAND(rg != nullptr);
  const maliput::api::Lane* lane = rg->ById().GetLane(
      maliput::api::LaneId(lane_id));
  const delphyne::RailFollowerParams<double> params;
  delphyne::RailFollowerState<double> state;
  state.set_s(start_s);
  state.set_speed(kMaliputRailcarSpeed);
  return simulator->AddPriusMaliputRailcar(
      "MaliputRailcar0",
      LaneDirection(lane, with_s),
      params,
      state,
      kAddLogger);
}

}  // namespace

int DoMain(int argc, char* argv[]) {
  const DemoType demo = ParseDemoType(argc, argv);
  if (demo == DemoType::kInvalid) {
    PrintUsage();
    return 0;
  }
  PrintDemoInformation(demo);

  auto simulator = std::make_unique<delphyne::MalidriveAutomotiveSimulator>();

  const std::string rg_id = GetFileNameFromPath(
      kDemoInformation.at(demo).file_path);
  const maliput::api::RoadNetwork* rn = AddMalidriveRoadNetwork(
      utility::FindResource(kDemoInformation.at(demo).file_path), rg_id,
      simulator.get());

  AddMaliputRailCar(
      simulator.get(), rn->road_geometry(), kDemoInformation.at(demo).lane_id,
      kDemoInformation.at(demo).s, kDemoInformation.at(demo).with_s);
  std::cout << "Car created and added to the simulator." << std::endl;


  std::cout << "Simulator started, about to step " << kSimulationDuration
            << "s." << std::endl;
  simulator->Start(kTargetRealTime);

  std::cout << "Simulator started, about to step " << kSimulationDuration
            << "s." << std::endl;
  simulator->StepBy(kSimulationDuration);

  std::cout << "Simulation ends." << std::endl;

  return 0;
}

}  // namespace delphyne


int main(int argc, char* argv[]) { return delphyne::DoMain(argc, argv); }
