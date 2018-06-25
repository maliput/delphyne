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
#include "helpers.h"
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
namespace {

struct LinkInfo {
  LinkInfo(std::string name_in, int robot_num_in, int num_geom_in)
      : name(name_in), robot_num(robot_num_in), num_geom(num_geom_in) {}
  std::string name;
  int robot_num{};
  int num_geom{};
};

// Returns the Prius link count.
int GetPriusLinkCount() { return PriusVis<double>(0, "").num_poses(); }

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

  EXPECT_EQ(link_count, GetPriusLinkCount());
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

}  // namespace

/*****************************************************************************
 * Tests
 ****************************************************************************/

// Fixture class for share configuration among all tests.
// Define Setup() if you need to set env variables and the like
class AutomotiveSimulatorTest : public ::testing::Test {
 protected:
  const double kSmallTimeStep{0.01};
  const double kLargeTimeStep{1.0};
  const double kRealtimeFactor{10.};
  const std::chrono::milliseconds kTimeoutMs{500};
};

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

  auto agent_bob = std::make_unique<delphyne::SimpleCar>(
      "bob", 0.0, 0.0, 0.0, 0.0);
  const int agent_bob_id = simulator->AddAgent(std::move(agent_bob));
  EXPECT_EQ(simulator->GetAgentById(agent_bob_id).name(), "bob");

  auto agent_alice = std::make_unique<delphyne::SimpleCar>(
      "alice", 0.0, 0.0, 0.0, 0.0);
  const int agent_alice_id = simulator->AddAgent(std::move(agent_alice));
  EXPECT_EQ(simulator->GetAgentById(agent_alice_id).name(), "alice");

  // Verifies that passing an unknown agent ID is an error.
  const int agent_x_id = -1;
  EXPECT_THROW(simulator->GetAgentById(agent_x_id), std::runtime_error);
}

// Covers simple-car, Start and StepBy
TEST_F(AutomotiveSimulatorTest, TestPriusSimpleCar) {
  // Set up a basic simulation with just a Prius SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  auto agent = std::make_unique<delphyne::SimpleCar>("bob", 0.0, 0.0, 0.0, 0.0);
  const int id = simulator->AddAgent(std::move(agent));

  EXPECT_EQ(id, 0);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start(kRealtimeFactor);

  // Simulate an external system sending a driving command to the car at
  // full throttle
  using ignition::msgs::AutomotiveDrivingCommand;
  const std::string kTeleopTopicName{"teleop/bob"};
  ignition::transport::Node node;
  auto publisher = node.Advertise<AutomotiveDrivingCommand>(kTeleopTopicName);

  AutomotiveDrivingCommand ign_msg;
  ign_msg.mutable_time()->set_sec(0);
  ign_msg.mutable_time()->set_nsec(0);
  ign_msg.set_acceleration(11.0);
  ign_msg.set_theta(0);
  publisher.Publish(ign_msg);

  // Set up a monitor to check for ignition::msgs::SimpleCarState
  // messages coming from the agent.
  const std::string kStateTopicName{"agents/0/state"};
  test::IgnMonitor<ignition::msgs::SimpleCarState> ign_monitor(kStateTopicName);

  // Shortly after starting, we should have not have moved much.
  const int kStateMessagesCount{1};
  EXPECT_TRUE(ign_monitor.do_until(
      kStateMessagesCount, kTimeoutMs,
      [this, &simulator]() { simulator->StepBy(kSmallTimeStep); }));

  ignition::msgs::SimpleCarState state_message = ign_monitor.get_last_message();
  EXPECT_LT(state_message.x(), 0.1);

  // Move a lot. Confirm that we're moving in +x.
  simulator->StepBy(kLargeTimeStep);

  state_message = ign_monitor.get_last_message();
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

  // Set up a monitor to check for ignition::msgs::SimpleCarState
  // messages coming from the agent.
  const std::string kStateTopicName{"agents/0/state"};
  test::IgnMonitor<ignition::msgs::SimpleCarState> ign_monitor(kStateTopicName);

  simulator->Start(kRealtimeFactor);

  const int kStateMessagesCount{1};
  EXPECT_TRUE(ign_monitor.do_until(
      kStateMessagesCount, kTimeoutMs,
      [this, &simulator]() { simulator->StepBy(kSmallTimeStep); }));

  const ignition::msgs::SimpleCarState state_message =
      ign_monitor.get_last_message();
  EXPECT_EQ(state_message.x(), kX);
  EXPECT_EQ(state_message.y(), kY);
  EXPECT_EQ(state_message.heading(), kHeading);
  EXPECT_EQ(state_message.velocity(), kVelocity);
}

TEST_F(AutomotiveSimulatorTest, TestMobilControlledSimpleCar) {
  // Set up a basic simulation with a MOBIL- and IDM-controlled SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  const drake::maliput::api::RoadGeometry* road_geometry{};
  EXPECT_NO_THROW({
    road_geometry = simulator->SetRoadGeometry(CreateDragway("TestDragway", 2));
  });

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
                                                    10.0,  // velocity
                                                    *road_geometry);
  const int id_mobil = simulator->AddAgent(std::move(mobil));
  EXPECT_EQ(0, id_mobil);

  auto decoy_1 = std::make_unique<delphyne::RailCar>(
      "decoy1", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      6.0,   // position (m)
      0.0,   // offset (m)
      0.0,   // speed (m)
      0.0,   // nominal_speed (m/s)
      *road_geometry);
  const int id_decoy1 = simulator->AddAgent(std::move(decoy_1));
  EXPECT_EQ(1, id_decoy1);

  auto decoy_2 = std::make_unique<delphyne::RailCar>(
      "decoy2", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      20.0,  // position (m)
      0.0,   // offset (m)
      0.0,   // speed (m/s)
      0.0,   // nominal_speed (m/s)
      *road_geometry);
  const int id_decoy2 = simulator->AddAgent(std::move(decoy_2));
  EXPECT_EQ(2, id_decoy2);

  // Setup an ignition transport topic monitor to listen to
  // ignition::msgs::Model_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  test::IgnMonitor<ignition::msgs::Model_V> ign_monitor(kDrawTopicName);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start(kRealtimeFactor);

  // Advances the simulation to allow the MaliputRailcar to begin
  // accelerating.
  simulator->StepBy(kLargeTimeStep);

  // Ensures that at least one draw message has arrived.
  const int kDrawMessagesCount{1};
  EXPECT_TRUE(ign_monitor.wait_until(kDrawMessagesCount, kTimeoutMs));

  const ignition::msgs::Model_V draw_message = ign_monitor.get_last_message();
  EXPECT_EQ(GetLinkCount(draw_message), 3 * GetPriusLinkCount());

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

  // Setup an ignition transport topic monitor to listen to
  // ignition::msgs::Model_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  test::IgnMonitor<ignition::msgs::Model_V> ign_monitor(kDrawTopicName);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start(kRealtimeFactor);

  // Simulate for 10 seconds.
  simulator->StepBy(10.);

  // Ensures at least one draw message has arrived.
  const int kDrawMessagesCount{1};
  EXPECT_TRUE(ign_monitor.wait_until(kDrawMessagesCount, kTimeoutMs));

  const ignition::msgs::Model_V draw_message = ign_monitor.get_last_message();

  EXPECT_EQ(GetLinkCount(draw_message), GetPriusLinkCount());

  auto alice_model = draw_message.models(id);

  // Checks the car ids
  EXPECT_EQ(alice_model.id(), id);

  auto link = alice_model.link(0);

  // Checks the chassis_floor body of the first car.
  EXPECT_EQ(link.name(), "chassis_floor");

  EXPECT_NEAR(link.pose().position().x(),
              // PriusVis<double>::kVisOffset + 30.00 won't work
              // because the trajectory agent is splining it's
              // way along.
              31.409479141235352, kPoseXTolerance);
  EXPECT_NEAR(link.pose().position().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().position().z(), 0.37832599878311157, kTolerance);
  EXPECT_NEAR(link.pose().orientation().w(), 1, kTolerance);
  EXPECT_NEAR(link.pose().orientation().x(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().z(), 0, kTolerance);
}

TEST_F(AutomotiveSimulatorTest, TestBadRailcars) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  const drake::maliput::api::RoadGeometry* dragway =
      simulator->SetRoadGeometry(CreateDragway("TestDragway", 1));

  const double kR{0.5};

  // make sure the simulator has a road geometry
  auto agent_1 = std::make_unique<delphyne::RailCar>(
      "foo", *(dragway->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      kR,    // offset
      0.0,   // speed
      0.0,   // nominal_speed
      *dragway);
  EXPECT_ARGUMENT_THROW(simulator->AddAgent(std::move(agent_1)),
                        "Rail cars need a road geometry to drive on, make sure "
                        "the simulation is configured with one.");

  std::unique_ptr<const drake::maliput::dragway::RoadGeometry>
      different_dragway = CreateDragway("DifferentDragway", 2);

  EXPECT_ARGUMENT_THROW(
      std::make_unique<delphyne::RailCar>(
          "bar", *(different_dragway->junction(0)->segment(0)->lane(0)),
          true,  // lane_direction,
          0.0,   // position
          kR,    // offset
          0.0,   // speed
          0.0,   // nominal_speed
          *dragway),
      "The provided initial lane is not on the same road "
      "geometry as that used by the simulation");
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
      0.0,       // nominal_speed
      *road_geometry);
  const int id = simulator->AddAgent(std::move(agent));

  EXPECT_GE(id, 0);

  // Setup an ignition transport topic monitor to listen to
  // ignition::msgs::Model_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  test::IgnMonitor<ignition::msgs::Model_V> ign_monitor(kDrawTopicName);

  simulator->Start(kRealtimeFactor);

  simulator->StepBy(kLargeTimeStep);

  // Ensures that at least one draw message has arrived.
  const int kDrawMessagesCount{1};
  EXPECT_TRUE(ign_monitor.wait_until(kDrawMessagesCount, kTimeoutMs));

  // Retrieves last draw message.
  const ignition::msgs::Model_V draw_message = ign_monitor.get_last_message();

  // Verifies the acceleration is zero.
  CheckModelLinks(draw_message);

  // The following tolerance was determined empirically.
  const double kPoseXTolerance{1e-4};
  EXPECT_NEAR(GetXPosition(draw_message, k_offset),
              PriusVis<double>::kVisOffset, kPoseXTolerance);
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

  // Setup an ignition transport topic monitor to listen to
  // ignition::msgs::Model_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  test::IgnMonitor<ignition::msgs::Model_V> ign_monitor(kDrawTopicName);

  simulator->Start(kRealtimeFactor);

  const std::unique_ptr<ignition::msgs::Scene> scene = simulator->GetScene();

  int scene_link_count = 0;
  for (const ignition::msgs::Model& model : scene->model()) {
    if (model.name() == "world") {
      scene_link_count += 1;
    } else {
      scene_link_count += model.link_size();
    }
  }

  // Checks number of links in the robot message, should be
  // equal to the Prius link count plus world and road geometry.
  EXPECT_EQ(scene_link_count, 2 * GetPriusLinkCount() + 2);

  // Takes a large step to for at least two draw messages to get
  // published, as that ensures that draw_message will not be
  // further changed.
  simulator->StepBy(kLargeTimeStep);
  const int kDrawMessagesCount{2};
  EXPECT_TRUE(ign_monitor.wait_until(kDrawMessagesCount, kTimeoutMs));

  // Retrieves last draw message.
  const ignition::msgs::Model_V draw_message = ign_monitor.get_last_message();

  // Checks number of links in the viewer_draw message.
  EXPECT_EQ(GetLinkCount(draw_message), 2 * GetPriusLinkCount());
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
  EXPECT_RUNTIME_THROW(
      simulator->AddAgent(std::move(agent2)),
      "An agent named \"Model1\" already exists. It has id 0.");

  auto agent_1 = std::make_unique<delphyne::RailCar>(
      "FOO", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      0.0,   // offset
      0.0,   // speed
      5.0,   // nominal_speed
      *road_geometry);
  EXPECT_NO_THROW(simulator->AddAgent(std::move(agent_1)));

  auto agent_2 = std::make_unique<delphyne::RailCar>(
      "alice", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      0.0,   // offset
      0.0,   // speed
      5.0,   // nominal_speed
      *road_geometry);
  EXPECT_NO_THROW(simulator->AddAgent(std::move(agent_2)));

  auto agent_3 = std::make_unique<delphyne::RailCar>(
      "alice", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      0.0,   // offset
      0.0,   // speed
      5.0,   // nominal_speed
      *road_geometry);
  EXPECT_RUNTIME_THROW(simulator->AddAgent(std::move(agent_3)),
                       "An agent named \"alice\" already exists. It has id 2.");
}

// Verifies that the velocity outputs of the rail cars are connected to
// the PoseAggregator, which prevents a regression of #5894.
TEST_F(AutomotiveSimulatorTest, TestRailcarVelocityOutput) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  const drake::maliput::api::RoadGeometry* road_geometry =
      simulator->SetRoadGeometry(
          std::make_unique<const drake::maliput::dragway::RoadGeometry>(
              drake::maliput::api::RoadGeometryId("TestDragway"),
              1 /* num lanes */, 100 /* length */, 4 /* lane width */,
              1 /* shoulder width */, 5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */));

  const double kR{0.5};

  auto alice = std::make_unique<delphyne::RailCar>(
      "alice", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      5.0,   // position
      kR,    // offset
      1.0,   // speed
      5.0,   // nominal_speed
      *road_geometry);

  auto bob = std::make_unique<delphyne::RailCar>(
      "bob", *(road_geometry->junction(0)->segment(0)->lane(0)),
      true,  // lane_direction,
      0.0,   // position
      kR,    // offset
      0.0,   // speed
      0.0,   // nominal_speed
      *road_geometry);

  int alice_id = simulator->AddAgent(std::move(alice));
  int bob_id = simulator->AddAgent(std::move(bob));

  EXPECT_NO_THROW(simulator->Start());

  // Advances the simulation to allow Alice to move at fixed
  // speed and Bob to not move.
  simulator->StepBy(kLargeTimeStep);

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

// Tests that collision detection works as expected. To that
// end, it sets up a simulation with three (3) simple Prius cars
// and two (2) straight lanes. `Bob` and `Alice` drive in opposite
// directions, and `Smith` goes behind `Bob` but with a slight
// heading deviation towards the other lane. Eventually, `Alice`
// and `Smith` collide.
//
// +---------------------------------------------+
//
//                           +-------+  +-------+
//   <---------------------+ |  Bob  |  | Smith |
//                           +-------+  +-------+
//                                   <-/
// +----------------------------- <-/------------+
//                             <-/
//   +-------+              <-/
//   | Alice | +-----X---<-/------------------->
//   +-------+
//
// +---------------------------------------------+
TEST_F(AutomotiveSimulatorTest, TestGetCollisions) {
  const int kNumLanes{2};
  const double kZeroSOffset{0.};                     // in m
  const double kZeroROffset{0.};                     // in m
  const double kZeroHOffset{0.};                     // in m
  const double kHeadingEast{0.};                     // in rads
  const double kHeadingWest{M_PI};                   // in rads
  const double kHeadingDeviation{M_PI * 4. / 180.};  // in rads
  const double kCruiseSpeed{10.};                    // in m/s
  const double kCarDistance{5.};                     // in m

  // Instantiates a simulator.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  // Builds a two (2) lane dragway to populate the
  // simulation world with.
  const drake::maliput::api::RoadGeometry* road =
      simulator->SetRoadGeometry(CreateDragway("TestDragway", kNumLanes));

  // Retrieves references to both lanes. Below's indirections
  // are guaranteed to be safe by Maliput's Dragway implementation.
  const drake::maliput::api::Lane* first_lane =
      road->junction(0)->segment(0)->lane(0);
  const drake::maliput::api::Lane* second_lane =
      road->junction(0)->segment(0)->lane(1);

  // Configures agent `Bob`.
  const drake::maliput::api::LanePosition agent_bob_lane_position{
      kCarDistance, kZeroROffset, kZeroHOffset};
  const drake::maliput::api::GeoPosition agent_bob_geo_position =
      first_lane->ToGeoPosition(agent_bob_lane_position);
  auto agent_bob = std::make_unique<delphyne::SimpleCar>(
      "bob", agent_bob_geo_position.x(), agent_bob_geo_position.y(),
      kHeadingEast, kCruiseSpeed);
  simulator->AddAgent(std::move(agent_bob));  // Unused agent `Bob` id.

  // Configures agent `Alice`.
  const drake::maliput::api::LanePosition agent_alice_lane_position{
      second_lane->length(), kZeroROffset, kZeroHOffset};
  const drake::maliput::api::GeoPosition agent_alice_geo_position =
      second_lane->ToGeoPosition(agent_alice_lane_position);
  auto agent_alice = std::make_unique<delphyne::SimpleCar>(
      "alice", agent_alice_geo_position.x(), agent_alice_geo_position.y(),
      kHeadingWest, kCruiseSpeed);
  const int agent_alice_id = simulator->AddAgent(std::move(agent_alice));

  // Configures agent `Smith`.
  const drake::maliput::api::LanePosition agent_smith_lane_position{
      kZeroSOffset, kZeroROffset, kZeroHOffset};
  const drake::maliput::api::GeoPosition agent_smith_geo_position =
      first_lane->ToGeoPosition(agent_smith_lane_position);
  auto agent_smith = std::make_unique<delphyne::SimpleCar>(
      "smith", agent_smith_geo_position.x(), agent_smith_geo_position.y(),
      kHeadingEast + kHeadingDeviation, kCruiseSpeed);
  const int agent_smith_id = simulator->AddAgent(std::move(agent_smith));

  // Finishes initialization and starts the simulation.
  simulator->Start();

  // Verifies that no agent is in collision at the beginning
  // of the simulation.
  std::vector<std::pair<int, int>> agent_pairs_in_collision =
      simulator->GetCollisions();
  EXPECT_EQ(agent_pairs_in_collision.size(), 0);

  // Simulates forward in time.
  const double kTimeToCollision{5.};  // in sec
  simulator->StepBy(kTimeToCollision);

  // Checks that there was a collision and that the colliding
  // agents are the expected ones.
  agent_pairs_in_collision = simulator->GetCollisions();
  EXPECT_EQ(agent_pairs_in_collision.size(), 1);
  const int first_colliding_agent_id = agent_pairs_in_collision[0].first;
  const int second_colliding_agent_id = agent_pairs_in_collision[0].second;
  // Cannot make any assumption regarding pair order, see
  // delphyne::AutomotiveSimulator::GetCollisions().
  EXPECT_TRUE((first_colliding_agent_id == agent_alice_id &&
               second_colliding_agent_id == agent_smith_id) ||
              (first_colliding_agent_id == agent_smith_id &&
               second_colliding_agent_id == agent_alice_id));
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace delphyne
