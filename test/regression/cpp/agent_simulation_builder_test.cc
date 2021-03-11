// Copyright 2017 Toyota Research Institute

#include "backend/agent_simulation_builder.h"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <drake/common/find_resource.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram_context.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>

#include <gtest/gtest.h>

#include "agents/mobil_car.h"
#include "agents/rail_car.h"
#include "agents/simple_car.h"
#include "agents/trajectory_agent.h"
#include "agents/unicycle_car.h"
#include "delphyne/protobuf/agent_state.pb.h"
#include "delphyne/protobuf/agent_state_v.pb.h"
#include "delphyne/roads/road_builder.h"
#include "systems/lane_direction.h"
#include "systems/trajectory.h"
#include "test/test_config.h"
#include "test_utilities/helpers.h"
#include "visualization/prius_vis.h"
#include "visualization/simple_prius_vis.h"

namespace delphyne {

/*****************************************************************************
 * Convenience Methods / Classes
 ****************************************************************************/
namespace {

// Returns the Prius link count.
int GetPriusLinkCount() { return SimplePriusVis<double>(0, "").num_poses(); }

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
std::unique_ptr<const maliput::api::RoadGeometry> CreateDragway(const std::string& name, const int& number_of_lanes) {
  return roads::CreateDragway(name, number_of_lanes, 100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
                              5 /* maximum_height */, std::numeric_limits<double>::epsilon() /* linear_tolerance */,
                              std::numeric_limits<double>::epsilon() /* angular_tolerance */);
}

// Retrieves the chassis floor link from the model message.
const ignition::msgs::Link& GetChassisFloorLink(const ignition::msgs::Model& message) {
  auto it = std::find_if(message.link().begin(), message.link().end(),
                         [](const ignition::msgs::Link& link) { return (link.name() == "chassis_floor"); });
  DELPHYNE_DEMAND(it != message.link().end());
  return *it;
}

// Checks the message has the expected link count and includes the
// chassis floor link.
void CheckModelLinks(const ignition::msgs::Model_V& message) {
  const int link_count = GetLinkCount(message);
  EXPECT_EQ(link_count, GetPriusLinkCount());
  const ignition::msgs::Model& model = message.models(0);
  ASSERT_NE(std::find_if(model.link().begin(), model.link().end(),
                         [](const ignition::msgs::Link& link) { return (link.name() == "chassis_floor"); }),
            model.link().end());
}

// Returns the x-position of the vehicle based on an ignition::msgs::Model_V.
// It also checks that the y-position of the vehicle is equal to the provided y
// value.
double GetXPosition(const ignition::msgs::Model_V& message, double y) {
  const ignition::msgs::Link& link = GetChassisFloorLink(message.models(0));
  EXPECT_DOUBLE_EQ(link.pose().position().y(), y);
  return link.pose().position().x();
}

}  // namespace

/*****************************************************************************
 * Tests
 ****************************************************************************/

// Fixture class for share configuration among all tests.
// Define Setup() if you need to set env variables and the like
class AgentSimulationTest : public ::testing::Test {
 protected:
  const double kSmallTimeStep{0.01};
  const double kLargeTimeStep{1.0};
  const double kRealtimeFactor{10.};
  const std::chrono::milliseconds kTimeoutMs{500};
};

// Tests GetVisualScene to return the scene
TEST_F(AgentSimulationTest, TestGetVisualScene) {
  constexpr double kZeroX{0.0};
  constexpr double kZeroY{0.0};
  constexpr double kZeroHeading{0.0};
  constexpr double kZeroSpeed{0.0};

  AgentSimulationBuilder builder;
  builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  builder.AddAgent<SimpleCarBlueprint>("bob", kZeroX, kZeroY, kZeroHeading, kZeroSpeed);
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  std::unique_ptr<ignition::msgs::Scene> scene = simulation->GetVisualScene();

  std::map<std::string, std::pair<int, int>> expected_load{{"chassis_floor", {0, 1}},   {"body", {0, 1}},
                                                           {"left_wheel", {0, 1}},      {"right_wheel", {0, 1}},
                                                           {"left_wheel_rear", {0, 1}}, {"right_wheel_rear", {0, 1}},
                                                           {"surface", {1, 1}}};

  for (int i = 0; i < scene->model_size(); i++) {
    auto model = scene->model(i);
    for (int k = 0; k < model.link_size(); k++) {
      auto link = model.link(k);
      ASSERT_TRUE(expected_load.count(link.name()) > 0) << "'" << link.name() << "' is not an expected link!";
      int robot_num, num_geometries;
      std::tie(robot_num, num_geometries) = expected_load[link.name()];
      EXPECT_EQ(link.visual_size(), num_geometries);
      EXPECT_EQ(i, robot_num);
    }
  }
}

// Simple touches on the getters.
TEST_F(AgentSimulationTest, BasicTest) {
  constexpr double kZeroX{0.0};
  constexpr double kZeroY{0.0};
  constexpr double kZeroHeading{0.0};
  constexpr double kZeroSpeed{0.0};

  AgentSimulationBuilder builder;
  builder.AddAgent<SimpleCarBlueprint>("bob", kZeroX, kZeroY, kZeroHeading, kZeroSpeed);
  builder.AddAgent<SimpleCarBlueprint>("alice", kZeroX, kZeroY, kZeroHeading, kZeroSpeed);
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Verifies that agents are present in the simulation.
  EXPECT_EQ(simulation->GetAgentByName("bob").name(), "bob");
  EXPECT_EQ(simulation->GetAgentByName("alice").name(), "alice");
  // Verifies that passing an unknown agent name is an error.
  EXPECT_THROW(simulation->GetAgentByName("agent_x"), std::runtime_error);
}

// Covers simple-car, Start and StepBy
TEST_F(AgentSimulationTest, TestPriusSimpleCar) {
  constexpr double kZeroX{0.0};
  constexpr double kZeroY{0.0};
  constexpr double kZeroHeading{0.0};
  constexpr double kZeroSpeed{0.0};

  // Set up a basic simulation with just a Prius SimpleCar on a dragway.
  AgentSimulationBuilder builder;
  builder.SetTargetRealTimeRate(kRealtimeFactor);
  builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  builder.AddAgent<SimpleCarBlueprint>("bob", kZeroX, kZeroY, kZeroHeading, kZeroSpeed);
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

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

  // Set up a monitor to check for ignition::msgs::AgentState
  // messages coming from the agent.
  const std::string kStateTopicName{"/agents/state"};
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::AgentState_V>(kStateTopicName);

  // Shortly after starting, we should not have moved much.
  const int kStateMessagesCount{1};
  EXPECT_TRUE(ign_monitor->do_until(kStateMessagesCount, kTimeoutMs,
                                    [this, &simulation]() { simulation->StepBy(kSmallTimeStep); }));

  EXPECT_TRUE(ign_monitor->get_last_message().states_size() > 0);

  ignition::msgs::AgentState state_message = ign_monitor->get_last_message().states(0);
  EXPECT_LT(state_message.position().x(), 0.1);

  // Move a lot. Confirm that we're moving in +x.
  simulation->StepBy(kLargeTimeStep);

  state_message = ign_monitor->get_last_message().states(0);
  EXPECT_GT(state_message.position().x(), 1.0);
}

// Covers unicycle car, Start and StepBy
TEST_F(AgentSimulationTest, TestPriusUnicycleCar) {
  constexpr double kZeroX{0.0};
  constexpr double kZeroY{0.0};
  constexpr double kZeroHeading{0.0};
  constexpr double kZeroSpeed{0.0};
  const std::string kAgentName{"unicycle"};

  // Set up a basic simulation with just a Prius UnicycleCar on a dragway.
  AgentSimulationBuilder builder;
  builder.SetTargetRealTimeRate(kRealtimeFactor);
  builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  builder.AddAgent<UnicycleCarBlueprint>(kAgentName, kZeroX, kZeroY, kZeroHeading, kZeroSpeed);
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  UnicycleCarAgent* agent = dynamic_cast<UnicycleCarAgent*>(simulation->GetMutableAgentByName(kAgentName));

  // Simulate an external system sending a driving command to the car at
  // full throttle
  agent->SetAcceleration(11.0);
  agent->SetAngularRate(0.0);

  // Set up a monitor to check for ignition::msgs::AgentState
  // messages coming from the agent.
  const std::string kStateTopicName{"agents/state"};
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::AgentState_V>(kStateTopicName);

  const int kStateMessagesCount{1};
  EXPECT_TRUE(ign_monitor->do_until(kStateMessagesCount, kTimeoutMs,
                                    [this, &simulation]() { simulation->StepBy(kSmallTimeStep); }));

  EXPECT_TRUE(ign_monitor->get_last_message().states_size() > 0);

  ignition::msgs::AgentState state_message = ign_monitor->get_last_message().states(0);
  EXPECT_LT(state_message.position().x(), 0.1);

  // Move a lot. Confirm that we're moving in +x.
  simulation->StepBy(kLargeTimeStep);

  state_message = ign_monitor->get_last_message().states(0);
  EXPECT_GT(state_message.position().x(), 1.0);
}

// Tests the ability to initialize a SimpleCar to a non-zero initial state.
TEST_F(AgentSimulationTest, TestPriusSimpleCarInitialState) {
  constexpr double kX{10};
  constexpr double kY{5.5};
  constexpr double kHeading{M_PI_2};
  constexpr double kSpeed{4.5};

  // Set up a basic simulation with just a Prius SimpleCar on a dragway.
  AgentSimulationBuilder builder;
  builder.SetTargetRealTimeRate(kRealtimeFactor);
  builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  builder.AddAgent<SimpleCarBlueprint>("bob", kX, kY, kHeading, kSpeed);
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Set up a monitor to check for ignition::msgs::AgentState
  // messages coming from the agent.
  const std::string kStateTopicName{"agents/state"};
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::AgentState_V>(kStateTopicName);

  const int kStateMessagesCount{1};
  EXPECT_TRUE(ign_monitor->do_until(kStateMessagesCount, kTimeoutMs,
                                    [this, &simulation]() { simulation->StepBy(kSmallTimeStep); }));

  EXPECT_TRUE(ign_monitor->get_last_message().states_size() > 0);

  const ignition::msgs::AgentState state_message = ign_monitor->get_last_message().states(0);

  // Computations of AgentState from a PoseBundle incur minimal numerical
  // precision loss. Hence, a small tolerance is allowed when comparing the
  // values from the AgentState with the expected values. Unused values are
  // expected to be zero.
  // TODO(clalancette): During the upgrade to drake
  // commit 26deccd2bc66cddb5ea5d9da9732b70437cb6ab6, we had to change the
  // accuracy from 1e-15 to 0.05 to make the tests pass.  We should investigate
  // why and see if we can make that smaller again.
  const double kAccuracy = 0.05;

  EXPECT_NEAR(state_message.position().x(), kX, kAccuracy);
  EXPECT_NEAR(state_message.position().y(), kY, kAccuracy);
  EXPECT_EQ(state_message.position().z(), 0.0);

  EXPECT_EQ(state_message.orientation().roll(), 0.0);
  EXPECT_EQ(state_message.orientation().pitch(), 0.0);
  EXPECT_NEAR(state_message.orientation().yaw(), kHeading, kAccuracy);

  EXPECT_NEAR(state_message.linear_velocity().x(), kSpeed * cos(kHeading), kAccuracy);
  EXPECT_NEAR(state_message.linear_velocity().y(), kSpeed * sin(kHeading), kAccuracy);
  EXPECT_EQ(state_message.linear_velocity().z(), 0.0);

  EXPECT_EQ(state_message.angular_velocity().x(), 0.0);
  EXPECT_EQ(state_message.angular_velocity().y(), 0.0);
  EXPECT_EQ(state_message.angular_velocity().z(), 0.0);
}

TEST_F(AgentSimulationTest, TestMobilControlledSimpleCar) {
  // Set up a basic simulation with a MOBIL- and IDM-controlled SimpleCar.
  AgentSimulationBuilder builder;
  builder.SetTargetRealTimeRate(kRealtimeFactor);
  const maliput::api::RoadGeometry* road_geometry = builder.SetRoadGeometry(CreateDragway("TestDragway", 2));
  const maliput::api::Lane& first_lane = *(road_geometry->junction(0)->segment(0)->lane(0));

  // Create one MOBIL car and two stopped cars arranged as follows:
  //
  // ---------------------------------------------------------------
  // ^  +r, +y                                          | Decoy 2 |
  // |    -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
  // +---->  +s, +x  | MOBIL Car |   | Decoy 1 |
  // ---------------------------------------------------------------

  builder.AddAgent<MobilCarBlueprint>("MOBIL0",
                                      true,   // lane_direction,
                                      2.0,    // x
                                      -2.0,   // y
                                      0.0,    // heading
                                      10.0);  // velocity

  builder.AddAgent<RailCarBlueprint>("decoy1", first_lane,
                                     true,  // lane_direction,
                                     6.0,   // position (m)
                                     0.0,   // offset (m)
                                     0.0,   // speed (m)
                                     0.0);  // nominal_speed (m/s)

  builder.AddAgent<RailCarBlueprint>("decoy2", first_lane,
                                     true,  // lane_direction,
                                     20.0,  // position (m)
                                     0.0,   // offset (m)
                                     0.0,   // speed (m/s)
                                     0.0);  // nominal_speed (m/s)

  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Setup ignition transport topic monitors to listen to
  // ignition::msgs::Model_V and ignition::msgs::Pose_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  const std::string kPoseTopicName{"visualizer/pose_update"};
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Model_V>(kDrawTopicName);
  auto ign_pose_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Pose_V>(kPoseTopicName);

  // Advances the simulation to allow the MaliputRailcar to begin
  // accelerating.
  simulation->StepBy(kLargeTimeStep);

  // Ensures that at least one draw message has arrived.
  const int kDrawMessagesCount{1};
  EXPECT_TRUE(ign_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));
  EXPECT_TRUE(ign_pose_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));

  const ignition::msgs::Model_V draw_message = ign_monitor->get_last_message();
  const ignition::msgs::Pose_V pose_message = ign_pose_monitor->get_last_message();
  EXPECT_EQ(GetLinkCount(draw_message), 3 * GetPriusLinkCount());

  // Expect the SimpleCar to start steering to the left; y value increases.
  const ignition::msgs::Link& link = GetChassisFloorLink(draw_message.models(0));
  EXPECT_GE(link.pose().position().y(), -2.);

  // Check that Model_V and Pose_V messages have unique Ids and matching poses
  EXPECT_TRUE(test::CheckMsgTranslation(draw_message, pose_message));
}

TEST_F(AgentSimulationTest, TestTrajectoryAgent) {
  // Tolerance is rather large as the trajectory agent
  // is splining its way through.
  constexpr double kPoseXTolerance{0.1};
  constexpr double kTolerance{1e-8};

  AgentSimulationBuilder builder;
  builder.SetTargetRealTimeRate(kRealtimeFactor);
  builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  std::vector<double> times{0.0, 5.0, 10.0, 15.0, 20.0};
  std::vector<double> headings(5, 0.0);
  std::vector<std::vector<double>> translations{
      {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {30.0, 0.0, 0.0}, {60.0, 0.0, 0.0}, {100.0, 0.0, 0.0},
  };
  builder.AddAgent<TrajectoryAgentBlueprint>("alice", times, headings, translations);

  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Setup ignition transport topic monitors to listen to
  // ignition::msgs::Model_V and ignition::msgs::Pose_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  const std::string kPoseTopicName{"visualizer/pose_update"};
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Model_V>(kDrawTopicName);
  auto ign_pose_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Pose_V>(kPoseTopicName);

  // Simulate for 10 seconds.
  simulation->StepBy(10.);

  // Ensures at least one draw message has arrived.
  const int kDrawMessagesCount{1};
  EXPECT_TRUE(ign_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));
  EXPECT_TRUE(ign_pose_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));

  const ignition::msgs::Model_V draw_message = ign_monitor->get_last_message();
  const ignition::msgs::Pose_V pose_message = ign_pose_monitor->get_last_message();

  CheckModelLinks(draw_message);

  auto alice_model = draw_message.models(0);

  // Checks the car ids
  EXPECT_EQ(static_cast<int>(alice_model.id()), 0);

  // Look for the chassis_floor link of the first car.
  const ignition::msgs::Link& link = GetChassisFloorLink(alice_model);

  EXPECT_NEAR(link.pose().position().x(), 30.0, kPoseXTolerance);
  EXPECT_NEAR(link.pose().position().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().position().z(), 0.37832599878311157, kTolerance);
  EXPECT_NEAR(link.pose().orientation().w(), 1, kTolerance);
  EXPECT_NEAR(link.pose().orientation().x(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().z(), 0, kTolerance);

  // Check that Model_V and Pose_V messages have unique Ids and matching poses
  EXPECT_TRUE(test::CheckMsgTranslation(draw_message, pose_message));
}

TEST_F(AgentSimulationTest, TestBadRailcars) {
  AgentSimulationBuilder builder;

  auto road_geometry = CreateDragway("TestDragway", 1);
  const maliput::api::Lane& first_lane = *(road_geometry->junction(0)->segment(0)->lane(0));

  EXPECT_ARGUMENT_THROW(
      {
        builder.AddAgent<RailCarBlueprint>("foo", first_lane,
                                           true,  // lane_direction,
                                           0.0,   // position
                                           0.5,   // offset
                                           0.0,   // speed
                                           0.0);  // nominal_speed
      },
      "Rail cars need a road geometry to drive on, make sure "
      "the simulation is built with one.");

  builder.SetRoadGeometry(CreateDragway("AnotherTestDragway", 2));

  EXPECT_ARGUMENT_THROW(
      {
        builder.AddAgent<RailCarBlueprint>("foo", first_lane,
                                           true,  // lane_direction,
                                           0.0,   // position
                                           0.5,   // offset
                                           0.0,   // speed
                                           0.0);  // nominal_speed
      },
      "The provided initial lane is not on the same road "
      "geometry as that used by the simulation");
}

// Covers railcar behavior.
TEST_F(AgentSimulationTest, TestMaliputRailcar) {
  AgentSimulationBuilder builder;
  builder.SetTargetRealTimeRate(kRealtimeFactor);
  const maliput::api::RoadGeometry* road_geometry = builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  const maliput::api::Lane& lane = *(road_geometry->junction(0)->segment(0)->lane(0));
  const double k_offset{0.5};
  builder.AddAgent<RailCarBlueprint>("railcar", lane,
                                     true,      // lane_direction,
                                     0.0,       // position
                                     k_offset,  // offset
                                     0.0,       // speed
                                     0.0);      // nominal_speed
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Setup ignition transport topic monitors to listen to
  // ignition::msgs::Model_V and ignition::msgs::Pose_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  const std::string kPoseTopicName{"visualizer/pose_update"};
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Model_V>(kDrawTopicName);
  auto ign_pose_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Pose_V>(kPoseTopicName);

  simulation->StepBy(kLargeTimeStep);

  // Ensures that at least one draw message has arrived.
  const int kDrawMessagesCount{1};
  EXPECT_TRUE(ign_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));
  EXPECT_TRUE(ign_pose_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));

  // Retrieves last draw message.
  const ignition::msgs::Model_V draw_message = ign_monitor->get_last_message();
  const ignition::msgs::Pose_V pose_message = ign_pose_monitor->get_last_message();

  // Verifies the acceleration is zero.
  CheckModelLinks(draw_message);

  const double kPoseXTolerance{1e-8};
  EXPECT_NEAR(GetXPosition(draw_message, k_offset), 0., kPoseXTolerance);

  // Check that Model_V and Pose_V messages have unique Ids and matching poses
  EXPECT_TRUE(test::CheckMsgTranslation(draw_message, pose_message));
}

// Verifies that CarVisApplicator, PoseBundleToDrawMessage, and
// LcmPublisherSystem are instantiated in Simulation's Diagram and
// collectively result in the correct ignition messages being published.
TEST_F(AgentSimulationTest, TestLcmOutput) {
  AgentSimulationBuilder builder;
  builder.SetTargetRealTimeRate(kRealtimeFactor);
  builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  builder.AddAgent<SimpleCarBlueprint>("Model1", 0.0, 0.0, 0.0, 0.0);
  builder.AddAgent<SimpleCarBlueprint>("Model2", 0.0, 0.0, 0.0, 0.0);
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Setup ignition transport topic monitors to listen to
  // ignition::msgs::Model_V and ignition::msgs::Pose_V messages being published to
  // the visualizer.
  const std::string kDrawTopicName{"visualizer/scene_update"};
  const std::string kPoseTopicName{"visualizer/pose_update"};
  auto ign_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Model_V>(kDrawTopicName);
  auto ign_pose_monitor = test::MakeSharedIgnMonitor<ignition::msgs::Pose_V>(kPoseTopicName);

  const std::unique_ptr<const ignition::msgs::Scene> scene = simulation->GetVisualScene();

  int scene_link_count = 0;
  for (const ignition::msgs::Model& model : scene->model()) {
    scene_link_count += model.link_size();
  }

  // Checks number of links in the robot message, it should be
  // equal to twice the Prius link count plus road geometry.
  EXPECT_EQ(scene_link_count, 2 * GetPriusLinkCount() + 1);

  // Takes a large step to for at least two draw messages to get
  // published, as that ensures that draw_message will not be
  // further changed.
  simulation->StepBy(kLargeTimeStep);
  const int kDrawMessagesCount{2};
  EXPECT_TRUE(ign_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));
  EXPECT_TRUE(ign_pose_monitor->wait_until(kDrawMessagesCount, kTimeoutMs));

  // Retrieves last draw message.
  const ignition::msgs::Model_V draw_message = ign_monitor->get_last_message();
  const ignition::msgs::Pose_V pose_message = ign_pose_monitor->get_last_message();

  // Checks number of links in the viewer_draw message.
  EXPECT_EQ(GetLinkCount(draw_message), 2 * GetPriusLinkCount());

  // Check that Model_V and Pose_V messages have unique Ids and matching poses
  EXPECT_TRUE(test::CheckMsgTranslation(draw_message, pose_message));
}

// Verifies that exceptions are thrown if a vehicle with a non-unique name is
// added to the simulation.
TEST_F(AgentSimulationTest, TestDuplicateVehicleNameException) {
  AgentSimulationBuilder builder;

  const maliput::api::RoadGeometry* road_geometry = builder.SetRoadGeometry(CreateDragway("TestDragway", 1));

  EXPECT_NO_THROW(builder.AddAgent<SimpleCarBlueprint>("Model1", 0.0, 0.0, 0.0, 0.0));
  EXPECT_RUNTIME_THROW(builder.AddAgent<SimpleCarBlueprint>("Model1", 0.0, 0.0, 0.0, 0.0),
                       "An agent named \"Model1\" already exists.");

  const maliput::api::Lane& lane = *(road_geometry->junction(0)->segment(0)->lane(0));

  EXPECT_NO_THROW(builder.AddAgent<RailCarBlueprint>("FOO", lane,
                                                     true,   // lane_direction,
                                                     0.0,    // position
                                                     0.0,    // offset
                                                     0.0,    // speed
                                                     5.0));  // nominal_speed

  EXPECT_NO_THROW(builder.AddAgent<RailCarBlueprint>("alice", lane,
                                                     true,   // lane_direction,
                                                     0.0,    // position
                                                     0.0,    // offset
                                                     0.0,    // speed
                                                     5.0));  // nominal_speed

  EXPECT_RUNTIME_THROW(builder.AddAgent<RailCarBlueprint>("alice", lane,
                                                          true,  // lane_direction,
                                                          0.0,   // position
                                                          0.0,   // offset
                                                          0.0,   // speed
                                                          5.0),  // nominal_speed
                       "An agent named \"alice\" already exists.");
}

// Verifies that the velocity outputs of the rail cars are connected to
// the PoseAggregator, which prevents a regression of #5894.
TEST_F(AgentSimulationTest, TestRailcarVelocityOutput) {
  AgentSimulationBuilder builder;

  const maliput::api::RoadGeometry* road_geometry = builder.SetRoadGeometry(roads::CreateDragway(
      "TestDragway", 1 /* num lanes */, 100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
      5 /* maximum_height */, std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */));

  const maliput::api::Lane& lane = *(road_geometry->junction(0)->segment(0)->lane(0));

  const double kR{0.5};

  builder.AddAgent<RailCarBlueprint>("alice", lane,
                                     true,  // lane_direction,
                                     5.0,   // position
                                     kR,    // offset
                                     1.0,   // speed
                                     5.0);  // nominal_speed

  builder.AddAgent<RailCarBlueprint>("bob", lane,
                                     true,  // lane_direction,
                                     0.0,   // position
                                     kR,    // offset
                                     0.0,   // speed
                                     0.0);  // nominal_speed

  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Advances the simulation to allow Alice to move at fixed
  // speed and Bob to not move.
  simulation->StepBy(kLargeTimeStep);

  const int kAliceIndex{0};
  const int kBobIndex{1};

  // Verifies that the velocity within the PoseAggregator's PoseBundle output
  // is non-zero.
  const drake::systems::rendering::PoseBundle<double> poses = simulation->GetCurrentPoses();
  ASSERT_EQ(poses.get_num_poses(), 2);

  ASSERT_EQ(poses.get_model_instance_id(kAliceIndex), 0);
  ASSERT_EQ(poses.get_model_instance_id(kBobIndex), 1);
  const RailCar& alice = simulation->GetAgentByName<RailCar>("alice");
  ASSERT_EQ(poses.get_name(kAliceIndex), alice.name());
  const RailCar& bob = simulation->GetAgentByName<RailCar>("bob");
  ASSERT_EQ(poses.get_name(kBobIndex), bob.name());
  EXPECT_FALSE(poses.get_velocity(kAliceIndex).get_value().isZero());
  EXPECT_TRUE(poses.get_velocity(kBobIndex).get_value().isZero());
}

// Tests Build logic
TEST_F(AgentSimulationTest, TestBuild) {
  AgentSimulationBuilder builder;
  builder.SetRoadGeometry(CreateDragway("TestDragway", 1));
  builder.AddAgent<SimpleCarBlueprint>("Model1", 0.0, 0.0, 0.0, 0.0);
  builder.AddAgent<SimpleCarBlueprint>("Model2", 0.0, 0.0, 0.0, 0.0);
  EXPECT_NO_THROW(builder.Build());
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
TEST_F(AgentSimulationTest, TestGetCollisions) {
  const int kNumLanes{2};
  const double kZeroSOffset{0.};                     // in m
  const double kZeroROffset{0.};                     // in m
  const double kZeroHOffset{0.};                     // in m
  const double kHeadingEast{0.};                     // in rads
  const double kHeadingWest{M_PI};                   // in rads
  const double kHeadingDeviation{M_PI * 4. / 180.};  // in rads
  const double kCruiseSpeed{10.};                    // in m/s
  const double kCarDistance{5.};                     // in m

  // Starts building a simulation.
  AgentSimulationBuilder builder;

  // Builds a two (2) lane dragway to populate the
  // simulation world with.
  const maliput::api::RoadGeometry* road = builder.SetRoadGeometry(CreateDragway("TestDragway", kNumLanes));

  // Retrieves references to both lanes. Below's indirections
  // are guaranteed to be safe by Maliput's Dragway implementation.
  const maliput::api::Lane* first_lane = road->junction(0)->segment(0)->lane(0);
  const maliput::api::Lane* second_lane = road->junction(0)->segment(0)->lane(1);

  // Configures agent `Bob`.
  const maliput::api::LanePosition agent_bob_lane_position{kCarDistance, kZeroROffset, kZeroHOffset};
  const maliput::api::InertialPosition agent_bob_inertial_position =
      first_lane->ToInertialPosition(agent_bob_lane_position);
  builder.AddAgent<SimpleCarBlueprint>("bob", agent_bob_inertial_position.x(), agent_bob_inertial_position.y(),
                                       kHeadingEast, kCruiseSpeed);

  // Configures agent `Alice`.
  const maliput::api::LanePosition agent_alice_lane_position{second_lane->length(), kZeroROffset, kZeroHOffset};
  const maliput::api::InertialPosition agent_alice_inertial_position =
      second_lane->ToInertialPosition(agent_alice_lane_position);

  builder.AddAgent<SimpleCarBlueprint>("alice", agent_alice_inertial_position.x(), agent_alice_inertial_position.y(),
                                       kHeadingWest, kCruiseSpeed);

  // Configures agent `Smith`.
  const maliput::api::LanePosition agent_smith_lane_position{kZeroSOffset, kZeroROffset, kZeroHOffset};
  const maliput::api::InertialPosition agent_smith_inertial_position =
      first_lane->ToInertialPosition(agent_smith_lane_position);
  builder.AddAgent<SimpleCarBlueprint>("smith", agent_smith_inertial_position.x(), agent_smith_inertial_position.y(),
                                       kHeadingEast + kHeadingDeviation, kCruiseSpeed);

  // Builds the simulation.
  std::unique_ptr<AgentSimulation> simulation = builder.Build();

  // Verifies that no agent is in collision at the beginning
  // of the simulation.
  std::vector<AgentBaseCollision<double>> agent_collisions = simulation->GetCollisions();
  EXPECT_EQ(static_cast<int>(agent_collisions.size()), 0);

  // Simulates forward in time.
  const double kTimeToCollision{5.};  // in sec
  simulation->StepBy(kTimeToCollision);

  // Checks that there was a collision and that the colliding
  // agents are the expected ones.
  agent_collisions = simulation->GetCollisions();
  ASSERT_EQ(static_cast<int>(agent_collisions.size()), 1);
  const AgentBaseCollision<double>& collision = agent_collisions.front();

  // Cannot make any assumption regarding pair order, see
  // delphyne::Simulation::GetCollisions().
  EXPECT_TRUE((collision.agents.first->name() == "alice" && collision.agents.second->name() == "smith") ||
              (collision.agents.first->name() == "smith" && collision.agents.second->name() == "alice"));
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace delphyne
