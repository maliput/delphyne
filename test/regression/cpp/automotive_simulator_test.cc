// Copyright 2017 Toyota Research Institute

#include "backend/automotive_simulator.h"
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "drake/automotive/curve2.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/find_resource.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/rendering/pose_bundle.h"

#include "delphyne/protobuf/simple_car_state.pb.h"
#include "test/test_config.h"

#include "src/agents/mobil_car.h"
#include "src/agents/rail_car.h"
#include "src/agents/trajectory_car.h"

using drake::automotive::PriusVis;
using drake::automotive::Curve2;
using drake::automotive::SimpleCarState;
using drake::automotive::DrivingCommand;
using drake::automotive::MaliputRailcarState;
using drake::automotive::MaliputRailcarParams;
using drake::automotive::LaneDirection;

namespace delphyne {

struct LinkInfo {
  LinkInfo(std::string name_in, int robot_num_in, int num_geom_in)
      : name(name_in), robot_num(robot_num_in), num_geom(num_geom_in) {}
  std::string name;
  int robot_num{};
  int num_geom{};
};

// Returns the number of links present in the ignition::msgs::Model_V message
// passed as a parameter.
int GetLinkCount(const ignition::msgs::Model_V& message) {
  int link_count = 0;

  for (int i = 0; i < message.models_size(); ++i) {
    link_count += message.models(i).link_size();
  }

  return link_count;
}

// Fixture class for share configuration among all tests.
class AutomotiveSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Set the paths where agents can be found.
    const char* env = "DELPHYNE_AGENT_PLUGIN_PATH=" DELPHYNE_PROJECT_BINARY_DIR
                      "/src/agents:" DELPHYNE_PROJECT_BINARY_DIR
                      "/test/regression/cpp/agent_plugin";
    putenv(const_cast<char*>(env));
  }
};

// Tests GetScene to return the scene
TEST_F(AutomotiveSimulatorTest, TestGetScene) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  auto initial_state = std::make_unique<SimpleCarState<double>>();

  simulator->AddLoadableAgent("simple-car", "my_test_model",
                              std::move(initial_state), nullptr);
  simulator->Start();
  std::unique_ptr<ignition::msgs::Scene> scene = simulator->GetScene();

  const std::vector<LinkInfo> expected_load{
      LinkInfo("chassis_floor", 0, 1),
      LinkInfo("front_axle", 0, 1),
      LinkInfo("left_tie_rod_arm", 0, 2),
      LinkInfo("left_hub", 0, 1),
      LinkInfo("tie_rod", 0, 1),
      LinkInfo("left_wheel", 0, 3),
      LinkInfo("right_tie_rod_arm", 0, 2),
      LinkInfo("right_hub", 0, 1),
      LinkInfo("right_wheel", 0, 3),
      LinkInfo("rear_axle", 0, 1),
      LinkInfo("left_wheel_rear", 0, 3),
      LinkInfo("right_wheel_rear", 0, 3),
      LinkInfo("body", 0, 1),
      LinkInfo("front_lidar_link", 0, 1),
      LinkInfo("top_lidar_link", 0, 1),
      LinkInfo("rear_right_lidar_link", 0, 1),
      LinkInfo("rear_left_lidar_link", 0, 1),
      LinkInfo("world", 0, 0)};

  for (int i = 0; i < scene->model_size(); i++) {
    auto model = scene->model(i);
    for (int k = 0; k < model.link_size(); k++) {
      auto link = model.link(k);
      EXPECT_EQ(i, expected_load.at(k).robot_num);
      EXPECT_EQ(link.name(), expected_load.at(k).name);
      EXPECT_EQ(link.visual_size(), expected_load.at(k).num_geom);
    }
  }
}

// Simple touches on the getters.
TEST_F(AutomotiveSimulatorTest, BasicTest) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  EXPECT_NE(nullptr, simulator->get_builder());
}

// Covers simple-car, Start and StepBy
TEST_F(AutomotiveSimulatorTest, TestPriusSimpleCar) {
  ignition::msgs::SimpleCarState state_message;
  std::function<void(const ignition::msgs::SimpleCarState& ign_message)>
      callback =
          [&state_message](const ignition::msgs::SimpleCarState& ign_message) {
            state_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::SimpleCarState>("agents/0/state", callback);

  // Set up a basic simulation with just a Prius SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  auto initial_state = std::make_unique<SimpleCarState<double>>();

  const int id = simulator->AddLoadableAgent("simple-car", "Foo",
                                             std::move(initial_state), nullptr);
  EXPECT_EQ(id, 0);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Simulate an external system sending a driving command to the car at
  // full throttle
  auto publisher =
      node.Advertise<ignition::msgs::AutomotiveDrivingCommand>("teleop/0");

  ignition::msgs::AutomotiveDrivingCommand ignMsg;

  ignMsg.mutable_time()->set_sec(0);
  ignMsg.mutable_time()->set_nsec(0);
  ignMsg.set_acceleration(11.0);
  ignMsg.set_theta(0);

  publisher.Publish(ignMsg);

  // Shortly after starting, we should have not have moved much. Take two
  // small steps so that we get a publish a small time after zero (publish
  // occurs at the beginning of a step unless specific publishing times are
  // set).
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  EXPECT_GT(state_message.x(), 0.0);
  EXPECT_LT(state_message.x(), 0.001);

  // Move a lot.  Confirm that we're moving in +x.
  for (int i = 0; i < 100; ++i) {
    simulator->StepBy(0.01);
  }

  EXPECT_GT(state_message.x(), 1.0);
}

// Tests the ability to initialize a SimpleCar to a non-zero initial state.
TEST_F(AutomotiveSimulatorTest, TestPriusSimpleCarInitialState) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  const double kX{10};
  const double kY{5.5};
  const double kHeading{M_PI_2};
  const double kVelocity{4.5};

  auto initial_state = std::make_unique<SimpleCarState<double>>();
  initial_state->set_x(kX);
  initial_state->set_y(kY);
  initial_state->set_heading(kHeading);
  initial_state->set_velocity(kVelocity);

  simulator->AddLoadableAgent("simple-car", "My_Test_Model",
                              std::move(initial_state), nullptr);

  ignition::msgs::SimpleCarState state_message;
  std::function<void(const ignition::msgs::SimpleCarState& ign_message)>
      callback =
          [&state_message](const ignition::msgs::SimpleCarState& ign_message) {
            state_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::SimpleCarState>("agents/0/state", callback);

  simulator->Start();
  simulator->StepBy(1e-3);

  EXPECT_EQ(state_message.x(), kX);
  EXPECT_EQ(state_message.y(), kY);
  EXPECT_EQ(state_message.heading(), kHeading);
  EXPECT_EQ(state_message.velocity(), kVelocity);
}

TEST_F(AutomotiveSimulatorTest, TestMobilControlledSimpleCar) {
  // Set up a basic simulation with a MOBIL- and IDM-controlled SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  const drake::maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const drake::maliput::dragway::RoadGeometry>(
              drake::maliput::api::RoadGeometryId("TestDragway"),
              2 /* num lanes */, 100 /* length */, 4 /* lane width */,
              1 /* shoulder width */, 5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */)));

  // Create one MOBIL car and two stopped cars arranged as follows:
  //
  // ---------------------------------------------------------------
  // ^  +r, +y                                          | Decoy 2 |
  // |    -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
  // +---->  +s, +x  | MOBIL Car |   | Decoy 1 |
  // ---------------------------------------------------------------

  auto simple_car_state =
      std::make_unique<drake::automotive::SimpleCarState<double>>();
  simple_car_state->set_x(2.0);
  simple_car_state->set_y(-2.0);
  simple_car_state->set_velocity(10.0);

  auto mobil_params = std::make_unique<MobilCarAgentParams>(true);
  const int id_mobil = simulator->AddLoadableAgent(
      "mobil-car", "MOBIL0", std::move(simple_car_state), road,
      std::move(mobil_params));
  EXPECT_EQ(0, id_mobil);
  auto decoy_state1 = std::make_unique<MaliputRailcarState<double>>();
  decoy_state1->set_s(6.0);
  decoy_state1->set_speed(0.0);

  auto lane_direction1 = std::make_unique<drake::automotive::LaneDirection>(
      road->junction(0)->segment(0)->lane(0));

  auto start_params1 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();

  auto railcar_params1 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction1), std::move(start_params1));

  const int id_decoy1 =
      simulator->AddLoadableAgent("rail-car", "decoy1", std::move(decoy_state1),
                                  road, std::move(railcar_params1));
  EXPECT_EQ(1, id_decoy1);

  auto decoy_state2 = std::make_unique<MaliputRailcarState<double>>();
  decoy_state2->set_s(20.0);
  decoy_state2->set_speed(0.0);

  auto lane_direction2 = std::make_unique<drake::automotive::LaneDirection>(
      road->junction(0)->segment(0)->lane(1));

  auto start_params2 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();

  auto railcar_params2 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction2), std::move(start_params2));

  const int id_decoy2 =
      simulator->AddLoadableAgent("rail-car", "decoy2", std::move(decoy_state2),
                                  road, std::move(railcar_params2));
  EXPECT_EQ(2, id_decoy2);

  // Setup the an ignition callback to store the latest ignition::msgs::Model_V
  // that is published to /visualizer/scene_update.
  ignition::msgs::Model_V draw_message;

  std::function<void(const ignition::msgs::Model_V& ign_message)>
      viewer_draw_callback =
          [&draw_message](const ignition::msgs::Model_V& ign_message) {
            draw_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::Model_V>("visualizer/scene_update",
                                          viewer_draw_callback);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Advances the simulation to allow the MaliputRailcar to begin accelerating.
  simulator->StepBy(0.5);

  EXPECT_EQ(GetLinkCount(draw_message),
            3 * PriusVis<double>(0, "").num_poses());

  // Expect the SimpleCar to start steering to the left; y value increases.
  const double mobil_y =
      draw_message.models(id_mobil).link(0).pose().position().y();
  EXPECT_GE(mobil_y, -2.);
}

// Covers adding a prius trajectory car as with loadable agent.
// TODO(daniel.stonier) : re-enable once the agent has-a (new) drake trajectory
// agent is built
// TEST_F(AutomotiveSimulatorTest, TestPriusTrajectoryCar) {
//   typedef Curve2<double> Curve2d;
//   typedef Curve2d::Point2 Point2d;
//   const std::vector<Point2d> waypoints{
//       {0.0, 0.0}, {100.0, 0.0},
//   };
//   const double kTolerance{1e-8};
//   const double kPoseXTolerance{1e-6};

//   // Set up a basic simulation with a couple Prius TrajectoryCars. Both cars
//   // start at position zero; the first has a speed of 1 m/s, while the other
//   is
//   // stationary. They both follow a straight 100 m long line.
//   auto simulator = std::make_unique<AutomotiveSimulator<double>>(
//       std::make_unique<drake::lcm::DrakeMockLcm>());

//   auto curveAlice =
//       std::make_unique<drake::automotive::Curve2<double>>(waypoints);
//   auto paramsAlice =
//       std::make_unique<TrajectoryCarAgentParams>(std::move(curveAlice));
//   auto stateAlice =
//       std::make_unique<drake::automotive::TrajectoryCarState<double>>();
//   stateAlice->set_speed(1.0);
//   stateAlice->set_position(0.0);
//   const int id1 = simulator->AddLoadableAgent("trajectory-car", "alice",
//                                               std::move(stateAlice), nullptr,
//                                               std::move(paramsAlice));

//   auto curveBob =
//       std::make_unique<drake::automotive::Curve2<double>>(waypoints);
//   auto paramsBob =
//   std::make_unique<TrajectoryCarAgentParams>(std::move(curveBob));
//   auto stateBob =
//       std::make_unique<drake::automotive::TrajectoryCarState<double>>();
//   stateBob->set_speed(0.0);
//   stateBob->set_position(0.0);
//   const int id2 =
//       simulator->AddLoadableAgent("trajectory-car", "bob",
//       std::move(stateBob),
//                                   nullptr, std::move(paramsBob));

//   EXPECT_EQ(0, id1);
//   EXPECT_EQ(1, id2);

//   // Setup the an ignition callback to store the latest
//   ignition::msgs::Model_V
//   // that is published to /visualizer/scene_update.
//   ignition::msgs::Model_V draw_message;

//   std::function<void(const ignition::msgs::Model_V& ign_message)>
//       viewer_draw_callback =
//           [&draw_message](const ignition::msgs::Model_V& ign_message) {
//             draw_message = ign_message;
//           };

//   ignition::transport::Node node;

//   node.Subscribe<ignition::msgs::Model_V>("visualizer/scene_update",
//                                           viewer_draw_callback);

//   // Finish all initialization, so that we can test the post-init state.
//   simulator->Start();

//   // Simulate for one second.
//   for (int i = 0; i < 100; ++i) {
//     simulator->StepBy(0.01);
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }

//   // Plus one to include the world.
//   const int expected_num_links = PriusVis<double>(0, "").num_poses() * 2 + 1;

//   // Minus one to omit world, which remains still.
//   EXPECT_EQ(GetLinkCount(draw_message), expected_num_links - 1);

//   auto alice_model = draw_message.models(id1);
//   auto bob_model = draw_message.models(id2);

//   // Checks the car ids
//   EXPECT_EQ(alice_model.id(), id1);
//   EXPECT_EQ(bob_model.id(), id2);

//   auto link = alice_model.link(0);

//   // Checks the chassis_floor body of the first car.
//   EXPECT_EQ(link.name(), "chassis_floor");

//   EXPECT_NEAR(link.pose().position().x(), PriusVis<double>::kVisOffset +
//   0.99,
//               kPoseXTolerance);
//   EXPECT_NEAR(link.pose().position().y(), 0, kTolerance);
//   EXPECT_NEAR(link.pose().position().z(), 0.378326, kTolerance);
//   EXPECT_NEAR(link.pose().orientation().w(), 1, kTolerance);
//   EXPECT_NEAR(link.pose().orientation().x(), 0, kTolerance);
//   EXPECT_NEAR(link.pose().orientation().y(), 0, kTolerance);
//   EXPECT_NEAR(link.pose().orientation().z(), 0, kTolerance);

//   // Checks the chassis_floor body of the first car.
//   EXPECT_EQ(alice_model.link_size(), bob_model.link_size());

//   // Verifies that the first car is about 1 m ahead of the second car. This
//   is
//   // expected since the first car is traveling at 1 m/s for a second while
//   the
//   // second car is immobile.
//   for (int i = 0; i < alice_model.link_size(); i++) {
//     auto alice_link = alice_model.link(i);
//     auto bob_link = bob_model.link(i);
//     EXPECT_EQ(alice_link.name(), bob_link.name());
//     EXPECT_NEAR(alice_link.pose().position().x(),
//                 bob_link.pose().position().x() + 0.99, kPoseXTolerance);
//     EXPECT_NEAR(alice_link.pose().position().y(),
//                 bob_link.pose().position().y(), kTolerance);
//     EXPECT_NEAR(alice_link.pose().position().z(),
//                 bob_link.pose().position().z(), kTolerance);
//     EXPECT_NEAR(alice_link.pose().orientation().w(),
//                 bob_link.pose().orientation().w(), kTolerance);
//     EXPECT_NEAR(alice_link.pose().orientation().x(),
//                 bob_link.pose().orientation().x(), kTolerance);
//     EXPECT_NEAR(alice_link.pose().orientation().y(),
//                 bob_link.pose().orientation().y(), kTolerance);
//     EXPECT_NEAR(alice_link.pose().orientation().z(),
//                 bob_link.pose().orientation().z(), kTolerance);
//   }
// }

// Checks the message has the expected link count and includes the
// chassis floor as its first link.
void CheckModelLinks(const ignition::msgs::Model_V& message) {
  const int link_count = GetLinkCount(message);

  auto link = message.models(0).link(0);

  EXPECT_EQ(link_count, PriusVis<double>(0, "").num_poses());
  EXPECT_EQ(link.name(), "chassis_floor");
}

// Returns the x-position of the vehicle based on an ignition::msgs::Model_V.
// It also checks that the y-position of the vehicle is equal to the provided y
// value.
double GetXPosition(const ignition::msgs::Model_V& message, double y) {
  auto link = message.models(0).link(0);
  EXPECT_DOUBLE_EQ(link.pose().position().y(), y);
  return link.pose().position().x();
}

TEST_F(AutomotiveSimulatorTest, TestBadMaliputRailcars) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  const double kR{0.5};
  MaliputRailcarParams<double> params;
  params.set_r(kR);

  const drake::maliput::api::RoadGeometry* road{};

  auto lane_direction1 = std::make_unique<drake::automotive::LaneDirection>();
  auto state1 = std::make_unique<MaliputRailcarState<double>>();
  auto start_params1 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  auto railcar_params1 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction1), std::move(start_params1));

  const int id1 = simulator->AddLoadableAgent(
      "rail-car", "foo", std::move(state1), road, std::move(railcar_params1));

  EXPECT_LT(id1, 0);

  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const drake::maliput::dragway::RoadGeometry>(
              drake::maliput::api::RoadGeometryId("TestDragway"),
              1 /* num lanes */, 100 /* length */, 4 /* lane width */,
              1 /* shoulder width */, 5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */)));

  auto lane_direction2 = std::make_unique<drake::automotive::LaneDirection>();
  auto state2 = std::make_unique<MaliputRailcarState<double>>();
  auto start_params2 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  auto railcar_params2 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction2), std::move(start_params2));
  const int id2 = simulator->AddLoadableAgent(
      "rail-car", "bar", std::move(state2), road, std::move(railcar_params2));
  EXPECT_LT(id2, 0);

  const auto different_road =
      std::make_unique<const drake::maliput::dragway::RoadGeometry>(
          drake::maliput::api::RoadGeometryId("DifferentDragway"),
          2 /* num lanes */, 50 /* length */, 3 /* lane width */,
          2 /* shoulder width */, 5 /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */);

  auto lane_direction3 = std::make_unique<drake::automotive::LaneDirection>(
      different_road->junction(0)->segment(0)->lane(0));
  auto state3 = std::make_unique<MaliputRailcarState<double>>();
  auto start_params3 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  auto railcar_params3 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction3), std::move(start_params3));
  const int id3 = simulator->AddLoadableAgent(
      "rail-car", "baz", std::move(state3), road, std::move(railcar_params3));

  EXPECT_LT(id3, 0);
}

// Covers railcar behavior.
TEST_F(AutomotiveSimulatorTest, TestMaliputRailcar) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  const double kR{0.5};

  const drake::maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const drake::maliput::dragway::RoadGeometry>(
              drake::maliput::api::RoadGeometryId("TestDragway"),
              1 /* num lanes */, 100 /* length */, 4 /* lane width */,
              1 /* shoulder width */, 5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */)));

  auto lane_direction = std::make_unique<drake::automotive::LaneDirection>(
      road->junction(0)->segment(0)->lane(0));
  auto state = std::make_unique<MaliputRailcarState<double>>();
  auto start_params =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  start_params->set_r(kR);
  auto railcar_params = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction), std::move(start_params));
  const int id = simulator->AddLoadableAgent(
      "rail-car", "model", std::move(state), road, std::move(railcar_params));

  EXPECT_GE(id, 0);

  // Setup an ignition callback to store the latest ignition::msgs::Model_V
  // that is published to /visualizer/scene_update
  ignition::msgs::Model_V draw_message;

  std::function<void(const ignition::msgs::Model_V& ign_message)>
      viewer_draw_callback =
          [&draw_message](const ignition::msgs::Model_V& ign_message) {
            draw_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::Model_V>("visualizer/scene_update",
                                          viewer_draw_callback);

  simulator->Start();

  // Takes two steps to trigger the publishing of an LCM draw message.
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const double initial_x = PriusVis<double>::kVisOffset;
  // Verifies the acceleration is zero.

  CheckModelLinks(draw_message);

  // The following tolerance was determined empirically.
  const double kPoseXTolerance{1e-4};

  EXPECT_NEAR(GetXPosition(draw_message, kR), initial_x, kPoseXTolerance);
}

// Verifies that CarVisApplicator, PoseBundleToDrawMessage, and
// LcmPublisherSystem are instantiated in AutomotiveSimulator's Diagram and
// collectively result in the correct ignition messages being published.
TEST_F(AutomotiveSimulatorTest, TestLcmOutput) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  auto state1 = std::make_unique<SimpleCarState<double>>();
  auto state2 = std::make_unique<SimpleCarState<double>>();

  simulator->AddLoadableAgent("simple-car", "Model1", std::move(state1),
                              nullptr);

  simulator->AddLoadableAgent("simple-car", "Model2", std::move(state2),
                              nullptr);

  // Setup the an ignition callback to store the latest ignition::msgs::Model_V
  // that is published to /visualizer/scene_update
  ignition::msgs::Model_V draw_message;

  // Condition variable for critical section.
  std::condition_variable cv;
  // Mutex for critical section.
  std::mutex mtx;

  // This counter keeps track of the number of times that the
  // callback function has been called.
  int num_of_callback_calls = 0;

  std::function<void(const ignition::msgs::Model_V& ign_message)>
      viewer_draw_callback = [&draw_message, &mtx, &cv, &num_of_callback_calls](
          const ignition::msgs::Model_V& ign_message) {
        std::unique_lock<std::mutex> lck(mtx);
        draw_message = ign_message;
        // Increases the counter to keep track of how
        // many times the callback has been called.
        num_of_callback_calls++;
        // Notifies the main thread that the counter was increased.
        cv.notify_one();
      };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::Model_V>("visualizer/scene_update",
                                          viewer_draw_callback);

  // Plus one to include the world.
  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 2 + 1;

  simulator->Start();

  std::unique_ptr<ignition::msgs::Scene> scene = simulator->GetScene();

  int scene_link_count = 0;
  for (const ignition::msgs::Model& model : scene->model()) {
    if (model.name() == "world") {
      scene_link_count += 1;
    } else {
      scene_link_count += model.link_size();
    }
  }

  // Checks number of links in the robot message.
  EXPECT_EQ(scene_link_count, expected_num_links);

  // Runs a single simulation step.
  simulator->StepBy(1e-3);

  // Waits until the callback has been executed twice, as that
  // ensures that draw_message will not be further changed.
  std::unique_lock<std::mutex> lck(mtx);
  std::cv_status status;
  const uint32_t kTimeoutMillis = 500;

  while (num_of_callback_calls < 2) {
    // The condition variable will wait for the callback function to be
    // notified or will raise a test assertion if the timeout time is reached.
    // This prevents the test to fall into a possible deadlock state.
    status = cv.wait_for(lck, std::chrono::milliseconds(kTimeoutMillis));
    if (status == std::cv_status::timeout) {
      FAIL() << "Condition variable timed out after waiting for "
             << kTimeoutMillis << "ms.";
    }
  }

  // Checks number of links in the viewer_draw message.
  EXPECT_EQ(GetLinkCount(draw_message), expected_num_links - 1);
}

// Verifies that exceptions are thrown if a vehicle with a non-unique name is
// added to the simulation.
TEST_F(AutomotiveSimulatorTest, TestDuplicateVehicleNameException) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  auto state1 = std::make_unique<SimpleCarState<double>>();
  auto state2 = std::make_unique<SimpleCarState<double>>();
  EXPECT_NO_THROW(simulator->AddLoadableAgent("simple-car", "Model1",
                                              std::move(state1), nullptr));
  EXPECT_THROW(simulator->AddLoadableAgent("simple-car", "Model1",
                                           std::move(state2), nullptr),
               std::runtime_error);

  const double kR{0.5};
  const drake::maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const drake::maliput::dragway::RoadGeometry>(
              drake::maliput::api::RoadGeometryId("TestDragway"),
              1 /* num lanes */, 100 /* length */, 4 /* lane width */,
              1 /* shoulder width */, 5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */)));

  auto lane_direction4 = std::make_unique<drake::automotive::LaneDirection>(
      road->junction(0)->segment(0)->lane(0));
  auto state4 = std::make_unique<MaliputRailcarState<double>>();
  auto start_params4 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  start_params4->set_r(kR);
  auto railcar_params4 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction4), std::move(start_params4));
  EXPECT_NO_THROW(simulator->AddLoadableAgent(
      "rail-car", "FOO", std::move(state4), road, std::move(railcar_params4)));

  auto lane_direction5 = std::make_unique<drake::automotive::LaneDirection>(
      road->junction(0)->segment(0)->lane(0));
  auto state5 = std::make_unique<MaliputRailcarState<double>>();
  auto start_params5 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  start_params5->set_r(kR);
  auto railcar_params5 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction5), std::move(start_params5));
  EXPECT_NO_THROW(simulator->AddLoadableAgent("rail-car", "alice",
                                              std::move(state5), road,
                                              std::move(railcar_params5)));

  auto lane_direction6 = std::make_unique<drake::automotive::LaneDirection>(
      road->junction(0)->segment(0)->lane(0));
  auto state6 = std::make_unique<MaliputRailcarState<double>>();
  auto start_params6 =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  start_params6->set_r(kR);
  auto railcar_params6 = std::make_unique<RailCarAgentParams>(
      std::move(lane_direction6), std::move(start_params6));
  EXPECT_THROW(
      simulator->AddLoadableAgent("rail-car", "alice", std::move(state6), road,
                                  std::move(railcar_params6)),
      std::runtime_error);
}

// Verifies that the velocity outputs of the MaliputRailcars are connected to
// the PoseAggregator, which prevents a regression of #5894.
TEST_F(AutomotiveSimulatorTest, TestRailcarVelocityOutput) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  const drake::maliput::api::RoadGeometry* road = simulator->SetRoadGeometry(
      std::make_unique<const drake::maliput::dragway::RoadGeometry>(
          drake::maliput::api::RoadGeometryId("TestDragway"), 1 /* num lanes */,
          100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
          5 /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */));

  const double kR{0.5};

  auto alice_initial_state = std::make_unique<MaliputRailcarState<double>>();
  alice_initial_state->set_s(5.0);
  alice_initial_state->set_speed(1.0);
  auto alice_lane_direction =
      std::make_unique<drake::automotive::LaneDirection>(
          road->junction(0)->segment(0)->lane(0));
  auto alice_start_params =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  alice_start_params->set_r(kR);
  auto alice_railcar_params = std::make_unique<RailCarAgentParams>(
      std::move(alice_lane_direction), std::move(alice_start_params));

  const int alice_id = simulator->AddLoadableAgent(
      "rail-car", "Alice", std::move(alice_initial_state), road,
      std::move(alice_railcar_params));

  auto bob_initial_state = std::make_unique<MaliputRailcarState<double>>();
  auto bob_lane_direction = std::make_unique<drake::automotive::LaneDirection>(
      road->junction(0)->segment(0)->lane(0));
  auto bob_start_params =
      std::make_unique<drake::automotive::MaliputRailcarParams<double>>();
  bob_start_params->set_r(kR);
  auto bob_railcar_params = std::make_unique<RailCarAgentParams>(
      std::move(bob_lane_direction), std::move(bob_start_params));

  const int bob_id = simulator->AddLoadableAgent(
      "rail-car", "Bob", std::move(bob_initial_state), road,
      std::move(bob_railcar_params));

  EXPECT_NO_THROW(simulator->Start());

  // Advances the simulation to allow Alice's MaliputRailcar to move at fixed
  // speed and Bob's MaliputRailcar to not move.
  simulator->StepBy(1);

  const int kAliceIndex{0};
  const int kBobIndex{1};

  // Verifies that the velocity within the PoseAggregator's PoseBundle output is
  // non-zero.
  const drake::systems::rendering::PoseBundle<double> poses =
      simulator->GetCurrentPoses();
  ASSERT_EQ(poses.get_num_poses(), 2);
  ASSERT_EQ(poses.get_model_instance_id(kAliceIndex), alice_id);
  ASSERT_EQ(poses.get_model_instance_id(kBobIndex), bob_id);
  EXPECT_FALSE(poses.get_velocity(kAliceIndex).get_value().isZero());
  EXPECT_TRUE(poses.get_velocity(kBobIndex).get_value().isZero());
}

// Tests Build/Start logic
TEST_F(AutomotiveSimulatorTest, TestBuild) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  auto state1 = std::make_unique<SimpleCarState<double>>();
  auto state2 = std::make_unique<SimpleCarState<double>>();
  simulator->AddLoadableAgent("simple-car", "Model1", std::move(state1),
                              nullptr);
  simulator->AddLoadableAgent("simple-car", "Model2", std::move(state2),
                              nullptr);

  simulator->Build();
  EXPECT_FALSE(simulator->has_started());
  EXPECT_NO_THROW(simulator->GetDiagram());

  simulator->Start(0.0);
  EXPECT_TRUE(simulator->has_started());
  EXPECT_NO_THROW(simulator->GetDiagram());
}

// Tests Build/Start logic (calling Start only)
TEST_F(AutomotiveSimulatorTest, TestBuild2) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  auto state1 = std::make_unique<SimpleCarState<double>>();
  auto state2 = std::make_unique<SimpleCarState<double>>();
  simulator->AddLoadableAgent("simple-car", "Model1", std::move(state1),
                              nullptr);
  simulator->AddLoadableAgent("simple-car", "Model2", std::move(state2),
                              nullptr);

  simulator->Start(0.0);
  EXPECT_NO_THROW(simulator->GetDiagram());
}

// Tests that AddLoadableAgent basically works.
TEST_F(AutomotiveSimulatorTest, TestAddLoadableAgentBasic) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  ASSERT_EQ(0, simulator->AddLoadableAgent("simple-car", "my_test_model",
                                           nullptr, nullptr));
}

// Tests that AddLoadableAgent returns -1 when unable to find plugin.
TEST_F(AutomotiveSimulatorTest, TestAddLoadableAgentNonExistent) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  ASSERT_EQ(-1, simulator->AddLoadableAgent("NonExistentPlugin",
                                            "my_test_model", nullptr, nullptr));
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace delphyne
