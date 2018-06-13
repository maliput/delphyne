// Copyright 2017 Toyota Research Institute

#include "backend/automotive_simulator.h"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <drake/automotive/lane_direction.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/dragway/road_geometry.h>
#include <drake/automotive/prius_vis.h>
#include <drake/automotive/trajectory.h>
#include <drake/common/find_resource.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram_context.h>
#include <drake/systems/rendering/pose_bundle.h>

#include <gtest/gtest.h>

#include "agents/mobil_car.h"
#include "agents/rail_car.h"
#include "agents/simple_car.h"
#include "agents/trajectory_agent.h"
#include "delphyne/protobuf/simple_car_state.pb.h"
#include "test/test_config.h"

using drake::automotive::PriusVis;
using drake::automotive::DrivingCommand;
using drake::automotive::LaneDirection;
using drake::automotive::MaliputRailcarState;
using drake::automotive::MaliputRailcarParams;
using drake::automotive::PriusVis;

namespace delphyne {

/*****************************************************************************
 * Convenience Methods / Classes
 ****************************************************************************/

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

// Using a dragway for most of the tests since it's the simplest, least
// volatile of the maliput road backends - i.e. should be able to trust it
// (tested elsewhere) and we can focus on the business of testing delphyne
// and the agents. Note that by construction, we guarantee that a chain
// such as road_geometry->junction(0)->segment(0)->lane(0) which is used
// frequently in the tests below exists and does not need to be checked
// for a null pointer.
std::unique_ptr<const drake::maliput::dragway::RoadGeometry> CreateDragway(
    const std::string& name, const int& number_of_lanes) {
  return std::make_unique<const drake::maliput::dragway::RoadGeometry>(
      drake::maliput::api::RoadGeometryId(name), number_of_lanes,
      100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
      5 /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance
      */);
}

// Checks the message has the expected link count and includes the
// chassis floor as its first link.
void CheckModelLinks(const ignition::msgs::Model_V& message) {
  const int link_count = GetLinkCount(message);

  const ignition::msgs::Link& link = message.models(0).link(0);

  EXPECT_EQ(link_count, PriusVis<double>(0, "").num_poses());
  EXPECT_EQ(link.name(), "chassis_floor");
}

// Returns the x-position of the vehicle based on an ignition::msgs::Model_V.
// It also checks that the y-position of the vehicle is equal to the provided y
// value.
double GetXPosition(const ignition::msgs::Model_V& message, double y) {
  const ignition::msgs::Link& link = message.models(0).link(0);
  EXPECT_DOUBLE_EQ(link.pose().position().y(), y);
  return link.pose().position().x();
}

/*****************************************************************************
 * Tests
 ****************************************************************************/

// Fixture class for share configuration among all tests.
// Define Setup() if you need to set env variables and the like
class AutomotiveSimulatorTest : public ::testing::Test {};

// Tests GetScene to return the scene
TEST_F(AutomotiveSimulatorTest, TestGetScene) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  auto agent = std::make_unique<delphyne::SimpleCar>("bob", 0.0, 0.0, 0.0, 0.0);
  simulator->AddAgent(std::move(agent));
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
      LinkInfo("world", 0, 0),
      LinkInfo("surface", 0, 1)};

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
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  auto agent = std::make_unique<delphyne::SimpleCar>("bob", 0.0, 0.0, 0.0, 0.0);
  const int id = simulator->AddAgent(std::move(agent));

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
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  const double kX{10};
  const double kY{5.5};
  const double kHeading{M_PI_2};
  const double kVelocity{4.5};

  auto agent =
      std::make_unique<delphyne::SimpleCar>("bob", kX, kY, kHeading, kVelocity);
  const int id = simulator->AddAgent(std::move(agent));
  EXPECT_EQ(id, 0);

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
  const drake::maliput::api::RoadGeometry* road_geometry{};
  EXPECT_NO_THROW(road_geometry = simulator->SetRoadGeometry(
                      CreateDragway("TestDragway", 2)););

  // Create one MOBIL car and two stopped cars arranged as follows:
  //
  // ---------------------------------------------------------------
  // ^  +r, +y                                          | Decoy 2 |
  // |    -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
  // +---->  +s, +x  | MOBIL Car |   | Decoy 1 |
  // ---------------------------------------------------------------

  auto mobil = std::make_unique<delphyne::MobilCar>("MOBIL0",
                                                    true,  // lane_direction,
                                                    2.0,   // x
                                                    -2.0,  // y
                                                    0.0,   // heading
                                                    10.0   // velocity
                                                    );
  const int id_mobil = simulator->AddAgent(std::move(mobil));
  EXPECT_EQ(0, id_mobil);

  auto decoy_1 = std::make_unique<delphyne::RailCar>(
      "decoy1", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      6.0,   // position (m)
      0.0,   // offset (m)
      0.0,   // speed (m)
      0.0    // nominal_speed (m/s)
      );
  const int id_decoy1 = simulator->AddAgent(std::move(decoy_1));
  EXPECT_EQ(1, id_decoy1);

  auto decoy_2 = std::make_unique<delphyne::RailCar>(
      "decoy2", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      20.0,  // position (m)
      0.0,   // offset (m)
      0.0,   // speed (m/s)
      0.0    // nominal_speed (m/s)
      );
  const int id_decoy2 = simulator->AddAgent(std::move(decoy_2));
  EXPECT_EQ(2, id_decoy2);

  // Setup the an ignition callback to store the latest
  // ignition::msgs::Model_V
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

  // Advances the simulation to allow the MaliputRailcar to begin
  // accelerating.
  simulator->StepBy(0.5);

  EXPECT_EQ(GetLinkCount(draw_message),
            3 * PriusVis<double>(0, "").num_poses());

  // Expect the SimpleCar to start steering to the left; y value increases.
  const double mobil_y =
      draw_message.models(id_mobil).link(0).pose().position().y();
  EXPECT_GE(mobil_y, -2.);
}

TEST_F(AutomotiveSimulatorTest, TestTrajectoryAgent) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  //  std::vector<double> times{0.0, 5.0, 10.0, 15.0, 20.0};
  //  Eigen::Quaternion<double> zero_heading(
  //      Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitZ()));
  //  std::vector<Eigen::Quaternion<double>> orientations(5, zero_heading);
  //  std::vector<Eigen::Vector3d> translations{
  //      Eigen::Vector3d(0.0, 0.00, 0.00), Eigen::Vector3d(10.0, 0.00, 0.00),
  //      Eigen::Vector3d(30.0, 0.00, 0.00), Eigen::Vector3d(60.0, 0.00, 0.00),
  //      Eigen::Vector3d(100.0, 0.00, 0.00)};
  //  drake::automotive::AgentTrajectory trajectory =
  //      drake::automotive::AgentTrajectory::Make(times, orientations,
  //                                               translations);
  const double kPoseXTolerance{1e-6};
  const double kTolerance{1e-8};

  std::vector<double> times{0.0, 5.0, 10.0, 15.0, 20.0};
  std::vector<double> headings(5, 0.0);
  std::vector<std::vector<double>> translations{
      {0.0, 0.0, 0.0},  {10.0, 0.0, 0.0},  {30.0, 0.0, 0.0},
      {60.0, 0.0, 0.0}, {100.0, 0.0, 0.0},
  };
  std::unique_ptr<delphyne::TrajectoryAgent> alice =
      std::make_unique<delphyne::TrajectoryAgent>("alice", times, headings,
                                                  translations);

  const int id = simulator->AddAgent(std::move(alice));

  EXPECT_EQ(0, id);

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

  //  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();
  //
  // Simulate for 10 seconds...as fast as possible
  for (int i = 0; i < 1000; ++i) {
    simulator->StepBy(0.01);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Plus two to include the world and road geometry
  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 1 + 2;

  // Minus one to omit world, which remains still.
  EXPECT_EQ(GetLinkCount(draw_message), expected_num_links - 2);

  auto alice_model = draw_message.models(id);

  // Checks the car ids
  EXPECT_EQ(alice_model.id(), id);

  auto link = alice_model.link(0);

  // Checks the chassis_floor body of the first car.
  EXPECT_EQ(link.name(), "chassis_floor");

  EXPECT_NEAR(link.pose().position().x(),
              // PriusVis<double>::kVisOffset + 30.00,
              // ... doesn't exactly work because the trajectory agent is
              // splining it's way along?
              31.369480133056641, kPoseXTolerance);
  EXPECT_NEAR(link.pose().position().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().position().z(), 0.37832599878311157, kTolerance);
  EXPECT_NEAR(link.pose().orientation().w(), 1, kTolerance);
  EXPECT_NEAR(link.pose().orientation().x(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().z(), 0, kTolerance);
}

TEST_F(AutomotiveSimulatorTest, TestBadRailcars) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  std::unique_ptr<const drake::maliput::dragway::RoadGeometry> dragway =
      CreateDragway("TestDragway", 1);

  const double kR{0.5};

  // make sure the simulator has a road geometry
  auto agent_1 = std::make_unique<delphyne::RailCar>(
      "foo", *(dragway->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      kR,    // offset
      0.0,   // speed
      0.0    // nominal_speed
      );
  const int id1 = simulator->AddAgent(std::move(agent_1));
  EXPECT_LT(id1, 0);

  // sim is using a different road geometry
  simulator->SetRoadGeometry(std::move(dragway));
  std::unique_ptr<const drake::maliput::dragway::RoadGeometry>
      different_dragway = CreateDragway("DifferentDragway", 2);
  auto agent_2 = std::make_unique<delphyne::RailCar>(
      "bar", *(different_dragway->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      kR,    // offset
      0.0,   // speed
      0.0    // nominal_speed
      );
  const int id2 = simulator->AddAgent(std::move(agent_2));

  EXPECT_LT(id2, 0);
}

// Covers railcar behavior.
TEST_F(AutomotiveSimulatorTest, TestMaliputRailcar) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  auto road_geometry =
      simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  const double k_offset{0.5};
  auto agent = std::make_unique<delphyne::RailCar>(
      "model", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,      // lane_direction,
      0.0,       // position
      k_offset,  // offset
      0.0,       // speed
      0.0        // nominal_speed
      );
  const int id = simulator->AddAgent(std::move(agent));

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

  EXPECT_NEAR(GetXPosition(draw_message, k_offset), initial_x, kPoseXTolerance);
}

// Verifies that CarVisApplicator, PoseBundleToDrawMessage, and
// LcmPublisherSystem are instantiated in AutomotiveSimulator's Diagram and
// collectively result in the correct ignition messages being published.
TEST_F(AutomotiveSimulatorTest, TestLcmOutput) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  auto agent1 =
      std::make_unique<delphyne::SimpleCar>("Model1", 0.0, 0.0, 0.0, 0.0);
  auto agent2 =
      std::make_unique<delphyne::SimpleCar>("Model2", 0.0, 0.0, 0.0, 0.0);
  simulator->AddAgent(std::move(agent1));
  simulator->AddAgent(std::move(agent2));

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

  simulator->Start();

  const std::unique_ptr<ignition::msgs::Scene> scene = simulator->GetScene();

  int scene_link_count = 0;
  for (const ignition::msgs::Model& model : scene->model()) {
    if (model.name() == "world") {
      scene_link_count += 1;
    } else {
      scene_link_count += model.link_size();
    }
  }

  // Plus two to include the world and road geometry
  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 2 + 2;
  // Checks number of links in the robot message.
  EXPECT_EQ(scene_link_count, expected_num_links);

  // Waits until the callback has been executed twice, as that
  // ensures that draw_message will not be further changed.
  const uint32_t kTimeoutMillis = 500;
  std::unique_lock<std::mutex> lck(mtx);
  while (num_of_callback_calls < 2) {
    // Runs a single simulation step.
    simulator->StepBy(1e-3);
    // The condition variable will wait for the callback function to be
    // notified or will raise a test assertion if the timeout time is reached.
    // This prevents the test to fall into a possible deadlock state.
    std::cv_status status =
        cv.wait_for(lck, std::chrono::milliseconds(kTimeoutMillis));
    if (status == std::cv_status::timeout) {
      FAIL() << "Condition variable timed out after waiting for "
             << kTimeoutMillis << "ms.";
    }
  }

  // Checks number of links in the viewer_draw message.
  EXPECT_EQ(GetLinkCount(draw_message), expected_num_links - 2);
}

// Verifies that exceptions are thrown if a vehicle with a non-unique name is
// added to the simulation.
TEST_F(AutomotiveSimulatorTest, TestDuplicateVehicleNameException) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  const drake::maliput::api::RoadGeometry* road_geometry =
      simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  auto agent1 =
      std::make_unique<delphyne::SimpleCar>("Model1", 0.0, 0.0, 0.0, 0.0);
  auto agent2 =
      std::make_unique<delphyne::SimpleCar>("Model1", 0.0, 0.0, 0.0, 0.0);
  EXPECT_NO_THROW(simulator->AddAgent(std::move(agent1)));
  EXPECT_THROW(simulator->AddAgent(std::move(agent2)), std::runtime_error);

  auto agent_1 = std::make_unique<delphyne::RailCar>(
      "FOO", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      0.0,   // offset
      0.0,   // speed
      5.0    // nominal_speed
      );
  EXPECT_NO_THROW(simulator->AddAgent(std::move(agent_1)));

  auto agent_2 = std::make_unique<delphyne::RailCar>(
      "alice", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      0.0,   // offset
      0.0,   // speed
      5.0    // nominal_speed
      );
  EXPECT_NO_THROW(simulator->AddAgent(std::move(agent_2)));

  auto agent_3 = std::make_unique<delphyne::RailCar>(
      "alice", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      0.0,   // offset
      0.0,   // speed
      5.0    // nominal_speed
      );
  EXPECT_THROW(simulator->AddAgent(std::move(agent_3)), std::runtime_error);
}

// Verifies that the velocity outputs of the rail cars are connected to
// the PoseAggregator, which prevents a regression of #5894.
TEST_F(AutomotiveSimulatorTest, TestRailcarVelocityOutput) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  const drake::maliput::api::RoadGeometry* road = simulator->SetRoadGeometry(
      std::make_unique<const drake::maliput::dragway::RoadGeometry>(
          drake::maliput::api::RoadGeometryId("TestDragway"), 1 /* num lanes
          */,
          100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
          5 /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */));

  const double kR{0.5};

  auto alice = std::make_unique<delphyne::RailCar>(
      "alice", *(road->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      5.0,   // position
      kR,    // offset
      1.0,   // speed
      5.0    // nominal_speed
      );

  auto bob = std::make_unique<delphyne::RailCar>(
      "bob", *(road->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      kR,    // offset
      0.0,   // speed
      0.0    // nominal_speed
      );

  int alice_id = simulator->AddAgent(std::move(alice));
  int bob_id = simulator->AddAgent(std::move(bob));

  EXPECT_NO_THROW(simulator->Start());

  // Advances the simulation to allow Alice to move at fixed
  // speed and Bob to not move.
  simulator->StepBy(1);

  const int kAliceIndex{0};
  const int kBobIndex{1};

  // Verifies that the velocity within the PoseAggregator's PoseBundle output
  // is non-zero.
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
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  auto agent1 =
      std::make_unique<delphyne::SimpleCar>("Model1", 0.0, 0.0, 0.0, 0.0);
  auto agent2 =
      std::make_unique<delphyne::SimpleCar>("Model2", 0.0, 0.0, 0.0, 0.0);
  simulator->AddAgent(std::move(agent1));
  simulator->AddAgent(std::move(agent2));

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
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  auto agent1 =
      std::make_unique<delphyne::SimpleCar>("Model1", 0.0, 0.0, 0.0, 0.0);
  auto agent2 =
      std::make_unique<delphyne::SimpleCar>("Model2", 0.0, 0.0, 0.0, 0.0);
  simulator->AddAgent(std::move(agent1));
  simulator->AddAgent(std::move(agent2));

  simulator->Start(0.0);
  EXPECT_NO_THROW(simulator->GetDiagram());
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace delphyne
