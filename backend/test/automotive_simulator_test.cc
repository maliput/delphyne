// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.

// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:

// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "backend/automotive_simulator.h"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "drake/automotive/curve2.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_simple_car_state_t.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/rendering/pose_bundle.h"

using drake::automotive::PriusVis;
using drake::automotive::Curve2;
using drake::automotive::SimpleCarState;
using drake::automotive::DrivingCommand;
using drake::automotive::MaliputRailcarState;
using drake::automotive::MaliputRailcarParams;
using drake::automotive::LaneDirection;

namespace delphyne {
namespace backend {

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

// Tests GetRobotModel to return the initial robot model
GTEST_TEST(AutomotiveSimulatorTest, TestGetRobotModel) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  SimpleCarState<double> initial_state;

  simulator->AddPriusSimpleCar("my_test_model", "Channel1", initial_state);

  auto robot_model = simulator->GetRobotModel();

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

  for (int i = 0; i < robot_model->models_size(); i++) {
    auto model = robot_model->models(i);
    for (int k = 0; k < model.link_size(); k++) {
      auto link = model.link(k);
      EXPECT_EQ(i, expected_load.at(k).robot_num);
      EXPECT_EQ(link.name(), expected_load.at(k).name);
      EXPECT_EQ(link.visual_size(), expected_load.at(k).num_geom);
    }
  }
}

// Simple touches on the getters.
GTEST_TEST(AutomotiveSimulatorTest, BasicTest) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  EXPECT_NE(nullptr, simulator->get_lcm());
  EXPECT_NE(nullptr, simulator->get_builder());
}

// Obtains the serialized version of the last message transmitted on LCM channel
// @p channel. Uses @p translator to decode the message into @p result.
void GetLastPublishedSimpleCarState(
    const std::string& channel,
    const drake::systems::lcm::LcmAndVectorBaseTranslator& translator,
    const drake::lcm::DrakeMockLcm* mock_lcm, SimpleCarState<double>* result) {
  const std::vector<uint8_t>& message =
      mock_lcm->get_last_published_message(channel);
  translator.Deserialize(message.data(), message.size(), result);
}

// Covers AddPriusSimpleCar (and thus AddPublisher), Start, StepBy,
// GetSystemByName.
GTEST_TEST(AutomotiveSimulatorTest, TestPriusSimpleCar) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");

  ignition::msgs::SimpleCarState state_message;
  std::function<void(const ignition::msgs::SimpleCarState& ign_message)>
      callback =
          [&state_message](const ignition::msgs::SimpleCarState& ign_message) {
            state_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::SimpleCarState>("/0_SIMPLE_CAR_STATE",
                                                 callback);

  const std::string kCommandChannelName = "DRIVING_COMMAND";

  const std::string driving_command_name =
      drake::systems::lcm::LcmSubscriberSystem::make_name(kCommandChannelName);

  // Set up a basic simulation with just a Prius SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  const int id = simulator->AddPriusSimpleCar("Foo", kCommandChannelName);
  EXPECT_EQ(id, 0);

  // Grab the systems we want while testing GetBuilderSystemByName() in the
  // process.
  auto& command_sub = dynamic_cast<drake::systems::lcm::LcmSubscriberSystem&>(
      simulator->GetBuilderSystemByName(driving_command_name));

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Set full throttle.
  DrivingCommand<double> command;
  command.set_acceleration(11.0);  // Arbitrary large positive.
  drake::lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<drake::lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(nullptr, mock_lcm);
  std::vector<uint8_t> message_bytes;
  command_sub.get_translator().Serialize(0.0 /* time */, command,
                                         &message_bytes);
  mock_lcm->InduceSubscriberCallback(kCommandChannelName, &message_bytes[0],
                                     message_bytes.size());

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
  // TODO(jwnimmer-tri) Check the timestamp of the final publication.
  EXPECT_GT(state_message.x(), 1.0);

  // The subsystem pointers must not change.
  EXPECT_EQ(&simulator->GetDiagramSystemByName(driving_command_name),
            &command_sub);
}

// Tests the ability to initialize a SimpleCar to a non-zero initial state.
GTEST_TEST(AutomotiveSimulatorTest, TestPriusSimpleCarInitialState) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());
  const double kX{10};
  const double kY{5.5};
  const double kHeading{M_PI_2};
  const double kVelocity{4.5};

  SimpleCarState<double> initial_state;
  initial_state.set_x(kX);
  initial_state.set_y(kY);
  initial_state.set_heading(kHeading);
  initial_state.set_velocity(kVelocity);

  simulator->AddPriusSimpleCar("My Test Model", "Channel", initial_state);

  ignition::msgs::SimpleCarState state_message;
  std::function<void(const ignition::msgs::SimpleCarState& ign_message)>
      callback =
          [&state_message](const ignition::msgs::SimpleCarState& ign_message) {
            state_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::SimpleCarState>("/0_SIMPLE_CAR_STATE",
                                                 callback);

  simulator->Start();
  simulator->StepBy(1e-3);

  EXPECT_EQ(state_message.x(), kX);
  EXPECT_EQ(state_message.y(), kY);
  EXPECT_EQ(state_message.heading(), kHeading);
  EXPECT_EQ(state_message.velocity(), kVelocity);
}

GTEST_TEST(AutomotiveSimulatorTest, TestMobilControlledSimpleCar) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  // Set up a basic simulation with a MOBIL- and IDM-controlled SimpleCar.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  drake::lcm::DrakeMockLcm* lcm =
      dynamic_cast<drake::lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(lcm, nullptr);

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
  SimpleCarState<double> simple_car_state;
  simple_car_state.set_x(2);
  simple_car_state.set_y(-2);
  simple_car_state.set_velocity(10);
  const int id_mobil = simulator->AddMobilControlledSimpleCar(
      "mobil", true /* with_s */, simple_car_state);
  EXPECT_EQ(id_mobil, 0);

  MaliputRailcarState<double> decoy_state;
  decoy_state.set_s(6);
  decoy_state.set_speed(0);
  const int id_decoy1 = simulator->AddPriusMaliputRailcar(
      "decoy1", LaneDirection(road->junction(0)->segment(0)->lane(0)),
      MaliputRailcarParams<double>(), decoy_state);
  EXPECT_EQ(id_decoy1, 1);

  decoy_state.set_s(20);
  const int id_decoy2 = simulator->AddPriusMaliputRailcar(
      "decoy2", LaneDirection(road->junction(0)->segment(0)->lane(1)),
      MaliputRailcarParams<double>(), decoy_state);
  EXPECT_EQ(id_decoy2, 2);

  // Setup the an ignition callback to store the latest ignition::msgs::Model_V
  // that is published to DRAKE_VIEWER_DRAW.
  ignition::msgs::Model_V draw_message;

  std::function<void(const ignition::msgs::Model_V& ign_message)>
      viewer_draw_callback =
          [&draw_message](const ignition::msgs::Model_V& ign_message) {
            draw_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::Model_V>("/DRAKE_VIEWER_DRAW",
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

//// Cover AddTrajectoryCar (and thus AddPublisher).
GTEST_TEST(AutomotiveSimulatorTest, TestPriusTrajectoryCar) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{
      {0.0, 0.0}, {100.0, 0.0},
  };
  const Curve2d curve{waypoints};
  const double kTolerance{1e-8};
  const double kPoseXTolerance{1e-6};

  // Set up a basic simulation with a couple Prius TrajectoryCars. Both cars
  // start at position zero; the first has a speed of 1 m/s, while the other is
  // stationary. They both follow a straight 100 m long line.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  const int id1 = simulator->AddPriusTrajectoryCar("alice", curve, 1.0, 0.0);
  const int id2 = simulator->AddPriusTrajectoryCar("bob", curve, 0.0, 0.0);
  EXPECT_EQ(id1, 0);
  EXPECT_EQ(id2, 1);

  // Setup the an ignition callback to store the latest ignition::msgs::Model_V
  // that is published to DRAKE_VIEWER_DRAW.
  ignition::msgs::Model_V draw_message;

  std::function<void(const ignition::msgs::Model_V& ign_message)>
      viewer_draw_callback =
          [&draw_message](const ignition::msgs::Model_V& ign_message) {
            draw_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::Model_V>("/DRAKE_VIEWER_DRAW",
                                          viewer_draw_callback);

  // Finish all initialization, so that we can test the post-init state.
  simulator->Start();

  // Simulate for one second.
  for (int i = 0; i < 100; ++i) {
    simulator->StepBy(0.01);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Plus one to include the world.
  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 2 + 1;

  // Minus one to omit world, which remains still.
  EXPECT_EQ(GetLinkCount(draw_message), expected_num_links - 1);

  auto alice_model = draw_message.models(id1);
  auto bob_model = draw_message.models(id2);

  // Checks the car ids
  EXPECT_EQ(alice_model.id(), id1);
  EXPECT_EQ(bob_model.id(), id2);

  auto link = alice_model.link(0);

  // Checks the chassis_floor body of the first car.
  EXPECT_EQ(link.name(), "chassis_floor");

  EXPECT_NEAR(link.pose().position().x(), PriusVis<double>::kVisOffset + 0.99,
              kPoseXTolerance);
  EXPECT_NEAR(link.pose().position().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().position().z(), 0.378326, kTolerance);
  EXPECT_NEAR(link.pose().orientation().w(), 1, kTolerance);
  EXPECT_NEAR(link.pose().orientation().x(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().y(), 0, kTolerance);
  EXPECT_NEAR(link.pose().orientation().z(), 0, kTolerance);

  // Checks the chassis_floor body of the first car.
  EXPECT_EQ(alice_model.link_size(), bob_model.link_size());

  // Verifies that the first car is about 1 m ahead of the second car. This is
  // expected since the first car is traveling at 1 m/s for a second while the
  // second car is immobile.
  for (int i = 0; i < alice_model.link_size(); i++) {
    auto alice_link = alice_model.link(i);
    auto bob_link = bob_model.link(i);
    EXPECT_EQ(alice_link.name(), bob_link.name());
    EXPECT_NEAR(alice_link.pose().position().x(),
                bob_link.pose().position().x() + 0.99, kPoseXTolerance);
    EXPECT_NEAR(alice_link.pose().position().y(),
                bob_link.pose().position().y(), kTolerance);
    EXPECT_NEAR(alice_link.pose().position().z(),
                bob_link.pose().position().z(), kTolerance);
    EXPECT_NEAR(alice_link.pose().orientation().w(),
                bob_link.pose().orientation().w(), kTolerance);
    EXPECT_NEAR(alice_link.pose().orientation().x(),
                bob_link.pose().orientation().x(), kTolerance);
    EXPECT_NEAR(alice_link.pose().orientation().y(),
                bob_link.pose().orientation().y(), kTolerance);
    EXPECT_NEAR(alice_link.pose().orientation().z(),
                bob_link.pose().orientation().z(), kTolerance);
  }
}

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

// Covers AddMaliputRailcar().
GTEST_TEST(AutomotiveSimulatorTest, TestMaliputRailcar) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  drake::lcm::DrakeMockLcm* lcm =
      dynamic_cast<drake::lcm::DrakeMockLcm*>(simulator->get_lcm());
  ASSERT_NE(lcm, nullptr);
  const double kR{0.5};
  MaliputRailcarParams<double> params;
  params.set_r(kR);

  EXPECT_THROW(simulator->AddPriusMaliputRailcar("foo", LaneDirection()),
               std::runtime_error);

  const drake::maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const drake::maliput::dragway::RoadGeometry>(
              drake::maliput::api::RoadGeometryId("TestDragway"),
              1 /* num lanes */, 100 /* length */, 4 /* lane width */,
              1 /* shoulder width */, 5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */)));

  EXPECT_THROW(
      simulator->AddPriusMaliputRailcar("bar", LaneDirection(), params),
      std::runtime_error);

  const auto different_road =
      std::make_unique<const drake::maliput::dragway::RoadGeometry>(
          drake::maliput::api::RoadGeometryId("DifferentDragway"),
          2 /* num lanes */, 50 /* length */, 3 /* lane width */,
          2 /* shoulder width */, 5 /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */);

  EXPECT_THROW(simulator->AddPriusMaliputRailcar(
                   "bar", LaneDirection(
                              different_road->junction(0)->segment(0)->lane(0)),
                   params),
               std::runtime_error);

  const int id = simulator->AddPriusMaliputRailcar(
      "model_name", LaneDirection(road->junction(0)->segment(0)->lane(0)),
      params, MaliputRailcarState<double>() /* initial state */);
  EXPECT_EQ(id, 0);

  // Setup the an ignition callback to store the latest ignition::msgs::Model_V
  // that is published to DRAKE_VIEWER_DRAW
  ignition::msgs::Model_V draw_message;

  std::function<void(const ignition::msgs::Model_V& ign_message)>
      viewer_draw_callback =
          [&draw_message](const ignition::msgs::Model_V& ign_message) {
            draw_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::Model_V>("/DRAKE_VIEWER_DRAW",
                                          viewer_draw_callback);

  simulator->Start();

  // Takes two steps to trigger the publishing of an LCM draw message.
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const double initial_x = PriusVis<double>::kVisOffset;
  // Verifies the acceleration is zero even if
  // AutomotiveSimulator::SetMaliputRailcarAccelerationCommand() was not called.

  CheckModelLinks(draw_message);

  // The following tolerance was determined empirically.
  const double kPoseXTolerance{1e-4};

  EXPECT_NEAR(GetXPosition(draw_message, kR), initial_x, kPoseXTolerance);

  // Sets the commanded acceleration to be zero.
  simulator->SetMaliputRailcarAccelerationCommand(id, 0);
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  CheckModelLinks(draw_message);

  // Verifies that the vehicle hasn't moved yet. This is expected since the
  // commanded acceleration is zero.
  // The following tolerance was determined empirically.
  const double step_2_position = GetXPosition(draw_message, kR);

  EXPECT_NEAR(step_2_position, initial_x, kPoseXTolerance);

  // Sets the commanded acceleration to be 10 m/s^2.
  simulator->SetMaliputRailcarAccelerationCommand(id, 10);

  // Advances the simulation to allow the MaliputRailcar to begin accelerating.
  simulator->StepBy(0.005);
  simulator->StepBy(0.005);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  CheckModelLinks(draw_message);

  // Verifies that the MaliputRailcar has moved forward relative to prior to
  // the nonzero acceleration command being issued.
  EXPECT_LT(step_2_position, GetXPosition(draw_message, kR));
}

// Verifies that CarVisApplicator, PoseBundleToDrawMessage, and
// LcmPublisherSystem are instantiated in AutomotiveSimulator's Diagram and
// collectively result in the correct ignition messages being published.
GTEST_TEST(AutomotiveSimulatorTest, TestLcmOutput) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  simulator->AddPriusSimpleCar("Model1", "Channel1");
  simulator->AddPriusSimpleCar("Model2", "Channel2");

  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{Point2d{0, 0}, Point2d{1, 0}};
  const Curve2d curve{waypoints};
  simulator->AddPriusTrajectoryCar("alice", curve, 1 /* speed */,
                                   0 /* start time */);
  simulator->AddPriusTrajectoryCar("bob", curve, 1 /* speed */,
                                   0 /* start time */);

  // Setup the an ignition callback to store the latest ignition::msgs::Model_V
  // that is published to DRAKE_VIEWER_DRAW
  ignition::msgs::Model_V draw_message;

  std::function<void(const ignition::msgs::Model_V& ign_message)>
      viewer_draw_callback =
          [&draw_message](const ignition::msgs::Model_V& ign_message) {
            draw_message = ign_message;
          };

  ignition::transport::Node node;

  node.Subscribe<ignition::msgs::Model_V>("/DRAKE_VIEWER_DRAW",
                                          viewer_draw_callback);

  // Plus one to include the world.
  const int expected_num_links = PriusVis<double>(0, "").num_poses() * 4 + 1;

  simulator->Start();

  auto robot_model = simulator->GetRobotModel();

  int robot_model_link_count = 0;

  for (int i = 0; i < robot_model->models_size(); ++i) {
    if (robot_model->models(i).name() == "world") {
      robot_model_link_count += 1;
    } else {
      robot_model_link_count += robot_model->models(i).link_size();
    }
  }

  EXPECT_EQ(robot_model_link_count, expected_num_links);

  simulator->StepBy(1e-3);

  EXPECT_EQ(GetLinkCount(draw_message), expected_num_links - 1);
}

// Verifies that exceptions are thrown if a vehicle with a non-unique name is
// added to the simulation.
GTEST_TEST(AutomotiveSimulatorTest, TestDuplicateVehicleNameException) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  EXPECT_NO_THROW(simulator->AddPriusSimpleCar("Model1", "Channel1"));
  EXPECT_THROW(simulator->AddPriusSimpleCar("Model1", "foo"),
               std::runtime_error);

  typedef Curve2<double> Curve2d;
  typedef Curve2d::Point2 Point2d;
  const std::vector<Point2d> waypoints{Point2d{0, 0}, Point2d{1, 0}};
  const Curve2d curve{waypoints};

  EXPECT_NO_THROW(simulator->AddPriusTrajectoryCar(
      "alice", curve, 1 /* speed */, 0 /* start time */));
  EXPECT_THROW(simulator->AddPriusTrajectoryCar("alice", curve, 1 /* speed */,
                                                0 /* start time */),
               std::runtime_error);
  EXPECT_THROW(simulator->AddPriusTrajectoryCar("Model1", curve, 1 /* speed */,
                                                0 /* start time */),
               std::runtime_error);

  const MaliputRailcarParams<double> params;
  const drake::maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const drake::maliput::dragway::RoadGeometry>(
              drake::maliput::api::RoadGeometryId("TestDragway"),
              1 /* num lanes */, 100 /* length */, 4 /* lane width */,
              1 /* shoulder width */, 5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */)));
  EXPECT_NO_THROW(simulator->AddPriusMaliputRailcar(
      "Foo", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */));
  EXPECT_THROW(
      simulator->AddPriusMaliputRailcar(
          "alice", LaneDirection(road->junction(0)->segment(0)->lane(0)),
          params, MaliputRailcarState<double>() /* initial state */),
      std::runtime_error);
  EXPECT_THROW(
      simulator->AddPriusMaliputRailcar(
          "Model1", LaneDirection(road->junction(0)->segment(0)->lane(0)),
          params, MaliputRailcarState<double>() /* initial state */),
      std::runtime_error);
}

// Verifies that no exception is thrown when multiple IDM-controlled
// MaliputRailcar vehicles are simulated. This prevents a regression of #5886.
GTEST_TEST(AutomotiveSimulatorTest, TestIdmControllerUniqueName) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  const MaliputRailcarParams<double> params;
  const drake::maliput::api::RoadGeometry* road = simulator->SetRoadGeometry(
      std::make_unique<const drake::maliput::dragway::RoadGeometry>(
          drake::maliput::api::RoadGeometryId("TestDragway"), 1 /* num lanes */,
          100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
          5 /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */));
  simulator->AddIdmControlledPriusMaliputRailcar(
      "Alice", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */);
  simulator->AddIdmControlledPriusMaliputRailcar(
      "Bob", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */);

  EXPECT_NO_THROW(simulator->Start());
}

// Verifies that the velocity outputs of the MaliputRailcars are connected to
// the PoseAggregator, which prevents a regression of #5894.
GTEST_TEST(AutomotiveSimulatorTest, TestRailcarVelocityOutput) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  const MaliputRailcarParams<double> params;
  const drake::maliput::api::RoadGeometry* road = simulator->SetRoadGeometry(
      std::make_unique<const drake::maliput::dragway::RoadGeometry>(
          drake::maliput::api::RoadGeometryId("TestDragway"), 1 /* num lanes */,
          100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
          5 /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */));
  MaliputRailcarState<double> alice_initial_state;
  alice_initial_state.set_s(5);
  alice_initial_state.set_speed(1);
  const int alice_id = simulator->AddPriusMaliputRailcar(
      "Alice", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      alice_initial_state);
  const int bob_id = simulator->AddIdmControlledPriusMaliputRailcar(
      "Bob", LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      MaliputRailcarState<double>() /* initial state */);

  EXPECT_NO_THROW(simulator->Start());

  // Advances the simulation to allow Alice's MaliputRailcar to move at fixed
  // speed and Bob's MaliputRailcar to move under IDM control.
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
  EXPECT_FALSE(poses.get_velocity(kBobIndex).get_value().isZero());
}

// Tests Build/Start logic
GTEST_TEST(AutomotiveSimulatorTest, TestBuild) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  simulator->AddPriusSimpleCar("Model1", "Channel1");
  simulator->AddPriusSimpleCar("Model2", "Channel2");

  simulator->Build();
  EXPECT_FALSE(simulator->has_started());
  EXPECT_NO_THROW(simulator->GetDiagram());

  simulator->Start(0.0);
  EXPECT_TRUE(simulator->has_started());
  EXPECT_NO_THROW(simulator->GetDiagram());
}

// Tests Build/Start logic (calling Start only)
GTEST_TEST(AutomotiveSimulatorTest, TestBuild2) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  simulator->AddPriusSimpleCar("Model1", "Channel1");
  simulator->AddPriusSimpleCar("Model2", "Channel2");

  simulator->Start(0.0);
  EXPECT_NO_THROW(simulator->GetDiagram());
}

// Verifies that messages are no longer being published in LCM
// DRAKE_VIEWER_LOAD_ROBOT and DRAKE_VIEWER_DRAW channels.
GTEST_TEST(AutomotiveSimulatorTest, TestNoLcm) {
  drake::AddResourceSearchPath(std::string(std::getenv("DRAKE_INSTALL_PATH")) +
                               "/share/drake");
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<drake::lcm::DrakeMockLcm>());

  simulator->AddPriusSimpleCar("Model1", "Channel1");

  simulator->Start();
  simulator->StepBy(1e-3);

  const drake::lcm::DrakeLcmInterface* lcm = simulator->get_lcm();
  ASSERT_NE(lcm, nullptr);

  const drake::lcm::DrakeMockLcm* mock_lcm =
      dynamic_cast<const drake::lcm::DrakeMockLcm*>(lcm);
  ASSERT_NE(mock_lcm, nullptr);

  // There should be no message published in DRAKE_VIEWER_LOAD_ROBOT
  EXPECT_THROW(mock_lcm->get_last_published_message("DRAKE_VIEWER_LOAD_ROBOT"),
               std::runtime_error);

  // There should be no message published in DRAKE_VIEWER_DRAW
  EXPECT_THROW(mock_lcm->get_last_published_message("DRAKE_VIEWER_DRAW"),
               std::runtime_error);
}

static const char* env = "AGENT_PLUGIN_PATH=test/agent_plugin";

// Tests that AddLoadableCar basically works.
GTEST_TEST(AutomotiveSimulatorTest, TestAddLoadableCarBasic) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  std::map<std::string, linb::any> params;

  ASSERT_EQ(0, simulator->AddLoadableCar("LoadableExampleDouble", params,
                                         "my_test_model", nullptr));
}

// Tests that AddLoadableCar returns -1 when unable to find plugin.
GTEST_TEST(AutomotiveSimulatorTest, TestAddLoadableCarNonExistent) {
  ASSERT_EQ(0, putenv(const_cast<char*>(env)));
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  std::map<std::string, linb::any> params;

  ASSERT_EQ(-1, simulator->AddLoadableCar("NonExistentPlugin", params,
                                          "my_test_model", nullptr));
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace backend
}  // namespace delphyne
