// Copyright 2017 Toyota Research Institute

#include <iostream>

#include "backend/lcm_to_ign_translation.h"
#include "backend/test/helpers.h"

#include <gtest/gtest.h>

#include <ignition/msgs.hh>

namespace delphyne {
namespace backend {

//////////////////////////////////////////////////
/// \brief Test that an LCM draw message is properly
/// translated to an ignition Geometry message.
class ViewerDrawTest : public ::testing::Test {
 protected:
  drake::lcmt_viewer_draw drawMsg;
  ignition::msgs::Model_V models;

  void SetUp() override {
    drawMsg.timestamp = 123456;
    drawMsg.num_links = 2;
    drawMsg.link_name.resize(drawMsg.num_links);
    drawMsg.robot_num.resize(drawMsg.num_links);
    drawMsg.position.resize(drawMsg.num_links);
    drawMsg.quaternion.resize(drawMsg.num_links);

    drawMsg.link_name[0] = "robot_one_link_one";
    drawMsg.robot_num[0] = 1;
    drawMsg.position[0] = {1.0, 2.0, 3.0};
    drawMsg.quaternion[0] = {4.0, 3.0, 2.0, 1.0};

    drawMsg.link_name[1] = "robot_two_link_one";
    drawMsg.robot_num[1] = 2;
    drawMsg.position[1] = {4.0, 5.0, 6.0};
    drawMsg.quaternion[1] = {8.0, 7.0, 6.0, 5.0};
  }
};

//////////////////////////////////////////////////
/// \brief Check that the translation of a PosesStamp
/// message with zero poses behaves as expected
TEST_F(ViewerDrawTest, TestZeroPosesInPosesStamp) {
  // Resize drawMsg defined in SetUp to zero poses
  drawMsg.num_links = 0;
  drawMsg.link_name.resize(0);
  drawMsg.robot_num.resize(0);
  drawMsg.position.resize(0);
  drawMsg.quaternion.resize(0);
  ignition::msgs::Model_V ignMsg;
  lcmToIgn(drawMsg, &ignMsg);
  ASSERT_EQ(ignMsg.models_size(), 0);
}

//////////////////////////////////////////////////
/// \brief Check that the translation of a PosesStamp
/// message with only one pose behaves as expected
TEST_F(ViewerDrawTest, TestOnePoseInPosesStamp) {
  // Resize drawMsg defined in SetUp to single pose
  drawMsg.num_links = 1;
  drawMsg.link_name.resize(1);
  drawMsg.robot_num.resize(1);
  drawMsg.position.resize(1);
  drawMsg.quaternion.resize(1);
  ignition::msgs::Model_V ignMsg;
  lcmToIgn(drawMsg, &ignMsg);
  EXPECT_TRUE(test::CheckMsgTranslation(drawMsg, ignMsg));
}

//////////////////////////////////////////////////
/// \brief Check that the translation of a PosesStamp message
/// with more than one pose behaves as expected
TEST_F(ViewerDrawTest, TestThreePosesInPosesStamp) {
  // Resize drawMsg defined in SetUp to three poses
  drawMsg.num_links = 3;
  drawMsg.link_name.resize(3);
  drawMsg.robot_num.resize(3);
  drawMsg.position.resize(3);
  drawMsg.quaternion.resize(3);
  // Fill third link values
  drawMsg.link_name[2] = "robot_three_link_one";
  drawMsg.robot_num[2] = 3;
  drawMsg.position[2] = {7.0, 8.0, 9.0};
  drawMsg.quaternion[2] = {12.0, 11.0, 10.0, 9.0};
  ignition::msgs::Model_V ignMsg;
  lcmToIgn(drawMsg, &ignMsg);
  EXPECT_TRUE(test::CheckMsgTranslation(drawMsg, ignMsg));
}

//////////////////////////////////////////////////
/// \brief Test that a header's sec and nsec were
/// correctly calculated
TEST_F(ViewerDrawTest, TestTimeStamp) {
  lcmToIgn(drawMsg, &models);
  int secs = 123;
  int nsecs = 456000000;
  ASSERT_EQ(secs, models.header().stamp().sec());
  ASSERT_EQ(nsecs, models.header().stamp().nsec());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if at least one of the
///  defined position vectors doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionInPosition) {
  drawMsg.position[1] = {1.0, 2.0, 3.0, 4.0};
  EXPECT_THROW(lcmToIgn(drawMsg, &models), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if at least
/// one of the defined orientation vectors doesn't
/// have the correct size
TEST_F(ViewerDrawTest, TestExceptionInOrientation) {
  drawMsg.quaternion[1] = {4.0, 2.0, 1.0};
  EXPECT_THROW(lcmToIgn(drawMsg, &models), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// link_name vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfLinkNames) {
  drawMsg.link_name.resize(drawMsg.num_links + 1);
  EXPECT_THROW(lcmToIgn(drawMsg, &models), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// robot_num vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfRobotNum) {
  drawMsg.robot_num.resize(drawMsg.num_links - 1);
  EXPECT_THROW(lcmToIgn(drawMsg, &models), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// position vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfPositions) {
  drawMsg.position.resize(drawMsg.num_links - 1);
  EXPECT_THROW(lcmToIgn(drawMsg, &models), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// quaternion vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfQuaternions) {
  drawMsg.position.resize(drawMsg.num_links + 1);
  EXPECT_THROW(lcmToIgn(drawMsg, &models), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a
/// box is properly translated to an ignition Geometry message.
GTEST_TEST(BoxTest, TestBoxTranslation) {
  // Define LCM box geometry message
  drake::lcmt_viewer_geometry_data boxMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignBoxGeometry;
  // Fill LCM data
  boxMsg.type = boxMsg.BOX;
  boxMsg.num_float_data = 3;
  boxMsg.float_data.resize(boxMsg.num_float_data);
  boxMsg.float_data[0] = 3.888;
  boxMsg.float_data[1] = 0.753;
  boxMsg.float_data[2] = 0.031;

  // Translate from LCM to ignition
  lcmToIgn(boxMsg, &ignBoxGeometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ignBoxGeometry.has_box());
  ASSERT_EQ(ignition::msgs::Geometry::BOX, ignBoxGeometry.type());
  ASSERT_EQ(boxMsg.float_data[0], ignBoxGeometry.box().size().x());
  ASSERT_EQ(boxMsg.float_data[1], ignBoxGeometry.box().size().y());
  ASSERT_EQ(boxMsg.float_data[2], ignBoxGeometry.box().size().z());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if an LCM geometry
/// message describing a box was not properly filled.
GTEST_TEST(BoxTest, TestExceptionInBoxTranslation) {
  // Define LCM box geometry message
  drake::lcmt_viewer_geometry_data boxMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignBoxGeometry;
  // Fill LCM data, but with missing z information
  boxMsg.type = boxMsg.BOX;
  boxMsg.num_float_data = 2;
  boxMsg.float_data.resize(boxMsg.num_float_data);
  boxMsg.float_data[0] = 1;
  boxMsg.float_data[1] = 2;

  EXPECT_THROW(lcmToIgn(boxMsg, &ignBoxGeometry), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a cylinder
/// was properly translated to an ignition Geometry message.
GTEST_TEST(CylinderTest, TestCylinderTranslation) {
  // Define LCM cylinder geometry message
  drake::lcmt_viewer_geometry_data cylinderMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignCylinderGeometry;
  // Fill LCM data
  cylinderMsg.type = cylinderMsg.CYLINDER;
  cylinderMsg.num_float_data = 2;
  cylinderMsg.float_data.resize(cylinderMsg.num_float_data);
  cylinderMsg.float_data[0] = 1.368;  // radius
  cylinderMsg.float_data[1] = 4.246;  // length

  // Translate from LCM to ignition
  lcmToIgn(cylinderMsg, &ignCylinderGeometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ignCylinderGeometry.has_cylinder());
  ASSERT_EQ(ignition::msgs::Geometry::CYLINDER, ignCylinderGeometry.type());
  ASSERT_EQ(cylinderMsg.float_data[0], ignCylinderGeometry.cylinder().radius());
  ASSERT_EQ(cylinderMsg.float_data[1], ignCylinderGeometry.cylinder().length());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if an LCM geometry
/// message describing a cylinder was not properly filled.
GTEST_TEST(CylinderTest, TestExceptionInCylinderTranslation) {
  // Define LCM cylinder geometry message
  drake::lcmt_viewer_geometry_data cylinderMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignCylinderGeometry;
  // Fill LCM data
  cylinderMsg.type = cylinderMsg.CYLINDER;
  cylinderMsg.num_float_data = 1;
  cylinderMsg.float_data.resize(cylinderMsg.num_float_data);
  cylinderMsg.float_data[0] = 1.368;  // radius

  EXPECT_THROW(lcmToIgn(cylinderMsg, &ignCylinderGeometry), TranslateException);
}

//////////////////////////////////////////////////
// \brief Test that an LCM geometry message describing a sphere
// was properly translated to an ignition Geometry message.
GTEST_TEST(SphereTest, TestSphereTranslation) {
  // Define LCM sphere geometry message
  drake::lcmt_viewer_geometry_data sphereMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignSphereGeometry;
  // Fill LCM data
  sphereMsg.type = sphereMsg.SPHERE;
  sphereMsg.num_float_data = 1;
  sphereMsg.float_data.resize(sphereMsg.num_float_data);
  sphereMsg.float_data[0] = 2.534;  // radius

  // Translate from LCM to ignition
  lcmToIgn(sphereMsg, &ignSphereGeometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ignSphereGeometry.has_sphere());
  ASSERT_EQ(ignition::msgs::Geometry::SPHERE, ignSphereGeometry.type());
  ASSERT_EQ(sphereMsg.float_data[0], ignSphereGeometry.sphere().radius());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if an LCM geometry
/// message describing a sphere was not properly filled.
GTEST_TEST(SphereTest, TestExceptionInCylinderTranslation) {
  // Define LCM sphere geometry message
  drake::lcmt_viewer_geometry_data sphereMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignSphereGeometry;
  // Fill LCM data
  sphereMsg.type = sphereMsg.SPHERE;
  sphereMsg.num_float_data = 0;
  sphereMsg.float_data.resize(sphereMsg.num_float_data);

  EXPECT_THROW(lcmToIgn(sphereMsg, &ignSphereGeometry), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a mesh with
/// scaling data was properly translated to an ignition Geometry message.
GTEST_TEST(MeshTest, TestMeshTranslationWithScale) {
  // Define LCM mesh geometry message
  drake::lcmt_viewer_geometry_data meshMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignMeshGeometry;
  // Fill LCM data
  meshMsg.type = meshMsg.MESH;
  meshMsg.string_data = "/path/to/mesh.obj";
  meshMsg.num_float_data = 3;
  meshMsg.float_data.resize(meshMsg.num_float_data);
  meshMsg.float_data[0] = 3.4;  // scale
  meshMsg.float_data[1] = 4.2;
  meshMsg.float_data[2] = 6.7;

  // Translate from LCM to ignition
  lcmToIgn(meshMsg, &ignMeshGeometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ignMeshGeometry.has_mesh());
  ASSERT_EQ(ignition::msgs::Geometry::MESH, ignMeshGeometry.type());
  ASSERT_EQ(meshMsg.string_data, ignMeshGeometry.mesh().filename());
  ASSERT_TRUE(ignMeshGeometry.mesh().has_scale());
  ASSERT_EQ(meshMsg.float_data[0], ignMeshGeometry.mesh().scale().x());
  ASSERT_EQ(meshMsg.float_data[1], ignMeshGeometry.mesh().scale().y());
  ASSERT_EQ(meshMsg.float_data[2], ignMeshGeometry.mesh().scale().z());
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a mesh without
/// scaling data is properly translated to an ignition Geometry message.
GTEST_TEST(MeshTest, TestMeshTranslationWithoutScale) {
  // Define LCM mesh geometry message
  drake::lcmt_viewer_geometry_data meshMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ignMeshGeometry;
  // Fill LCM data
  meshMsg.type = meshMsg.MESH;
  meshMsg.string_data = "/path/to/mesh.obj";
  meshMsg.num_float_data = 0;
  meshMsg.float_data.resize(meshMsg.num_float_data);

  // Translate from LCM to ignition
  lcmToIgn(meshMsg, &ignMeshGeometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ignMeshGeometry.has_mesh());
  ASSERT_EQ(ignition::msgs::Geometry::MESH, ignMeshGeometry.type());
  ASSERT_EQ(meshMsg.string_data, ignMeshGeometry.mesh().filename());
  ASSERT_FALSE(ignMeshGeometry.mesh().has_scale());
}

//////////////////////////////////////////////////
/// \brief Test that an LCM viewer link message describing a link
/// properly translates the name to an ignition Geometry message.
GTEST_TEST(MeshTest, TestLinkTranslationWithRobotNum) {
  drake::lcmt_viewer_link_data linkMsg;
  ignition::msgs::Link ignLink;

  linkMsg.name = "test_link";
  linkMsg.robot_num = 1234;
  linkMsg.num_geom = 0;

  lcmToIgn(linkMsg, &ignLink);

  ASSERT_EQ(NULL, ignLink.id());
  ASSERT_EQ("test_link", ignLink.name());
}

//////////////////////////////////////////////////
/// \brief Test that the id of a model is properly setup based
/// on the links ids
GTEST_TEST(ModelTest, TestModelIdTranslation) {
  drake::lcmt_viewer_load_robot robotData;
  drake::lcmt_viewer_link_data boxLink;

  ignition::msgs::Model robotModel;

  boxLink.name = "test_box";
  boxLink.robot_num = 12;
  boxLink.num_geom = 0;

  robotData.num_links = 1;
  robotData.link.resize(1);
  robotData.link[0] = boxLink;

  lcmToIgn(robotData, &robotModel);

  ASSERT_EQ(12, robotModel.id());
}

//////////////////////////////////////////////////
/// \brief Test that the ids of a set of models are properly setup based
/// on the links ids
GTEST_TEST(ModelTest, TestModelIdsTranslation) {
  drake::lcmt_viewer_load_robot robotData;
  drake::lcmt_viewer_link_data boxLink1;
  drake::lcmt_viewer_link_data boxLink2;
  drake::lcmt_viewer_link_data boxLink3;

  ignition::msgs::Model_V robotModels;

  boxLink1.name = "test_box_1";
  boxLink1.robot_num = 12;
  boxLink1.num_geom = 0;

  boxLink2.name = "test_box_2";
  boxLink2.robot_num = 8;
  boxLink2.num_geom = 0;

  boxLink3.name = "test_box_3";
  boxLink3.robot_num = 1;
  boxLink3.num_geom = 0;

  robotData.num_links = 3;
  robotData.link.resize(3);
  robotData.link[0] = boxLink1;
  robotData.link[1] = boxLink2;
  robotData.link[2] = boxLink3;

  lcmToIgn(robotData, &robotModels);

  EXPECT_EQ(1, robotModels.models(0).id());
  EXPECT_EQ(8, robotModels.models(1).id());
  EXPECT_EQ(12, robotModels.models(2).id());
}

//////////////////////////////////////////////////
/// \brief Test that the links geometries are properly assigned to the
/// different models
GTEST_TEST(ModelTest, TestModelsTranslation) {
  std::vector<float> quaternion = {1.0, 0.0, 0.0, 0.0};
  std::vector<float> color = {1.0, 1.0, 1.0, 1.0};

  drake::lcmt_viewer_load_robot robotData;

  drake::lcmt_viewer_link_data boxLink;
  drake::lcmt_viewer_geometry_data boxGeometry;

  drake::lcmt_viewer_link_data sphereLink;
  drake::lcmt_viewer_geometry_data sphereGeometry;

  ignition::msgs::Model_V robotModels;

  boxGeometry.type = boxGeometry.BOX;
  boxGeometry.num_float_data = 3;
  boxGeometry.float_data = {1, 2, 0.5};
  boxGeometry.position[0] = 0.0;
  boxGeometry.position[1] = 0.0;
  boxGeometry.position[2] = 0.0;
  std::copy(quaternion.begin(), quaternion.end(), boxGeometry.quaternion);
  std::copy(color.begin(), color.end(), boxGeometry.color);

  boxLink.name = "test_box";
  boxLink.robot_num = 3;
  boxLink.num_geom = 1;
  boxLink.geom.resize(1);
  boxLink.geom[0] = boxGeometry;

  sphereGeometry.type = sphereGeometry.SPHERE;
  sphereGeometry.num_float_data = 1;
  sphereGeometry.float_data.resize(1);
  sphereGeometry.float_data[0] = 1;
  sphereGeometry.position[0] = 3;
  sphereGeometry.position[1] = 3;
  sphereGeometry.position[2] = 3;
  std::copy(quaternion.begin(), quaternion.end(), sphereGeometry.quaternion);
  std::copy(color.begin(), color.end(), sphereGeometry.color);

  sphereLink.name = "test_sphere";
  sphereLink.robot_num = 5;
  sphereLink.num_geom = 1;
  sphereLink.geom.resize(1);
  sphereLink.geom[0] = sphereGeometry;

  robotData.num_links = 2;
  robotData.link.resize(2);
  robotData.link[0] = boxLink;
  robotData.link[1] = sphereLink;

  lcmToIgn(robotData, &robotModels);

  ASSERT_EQ(2, robotModels.models_size());

  auto boxModel = robotModels.models(0);
  EXPECT_EQ(3, boxModel.id());
  ASSERT_EQ(1, boxModel.link_size());
  EXPECT_EQ("test_box", boxModel.link(0).name());

  auto sphereModel = robotModels.models(1);
  EXPECT_EQ(5, sphereModel.id());
  ASSERT_EQ(1, sphereModel.link_size());
  EXPECT_EQ("test_sphere", sphereModel.link(0).name());
}

//////////////////////////////////////////////////
/// \brief Test that an LCM lcmt_viewer_command is
/// properly translated into an ignition ViewerCommand message
GTEST_TEST(ViewerCommandTest, TestViewerCommandTranslation) {
  drake::lcmt_viewer_command lcmViewerCommand;
  ignition::msgs::ViewerCommand ignViewerCommand;
  std::string testString = "test string";

  lcmViewerCommand.command_type = lcmViewerCommand.STATUS;
  lcmViewerCommand.command_data = testString;

  lcmToIgn(lcmViewerCommand, &ignViewerCommand);

  EXPECT_EQ(ignViewerCommand.STATUS, ignViewerCommand.command_type());
  EXPECT_EQ(testString, ignViewerCommand.command_data());
}

//////////////////////////////////////////////////
/// \brief Test that an LCM lcmt_simple_car_state_t message is
/// properly translated to a custom ignition SimpleCarState message.
GTEST_TEST(SimpleCarStateTest, TestSimpleCarStateTranslation) {
  drake::lcmt_simple_car_state_t lcmSimpleCarState;
  ignition::msgs::SimpleCarState ignSimpleCarState;

  lcmSimpleCarState.x = 2.56;
  lcmSimpleCarState.y = 8.39;
  lcmSimpleCarState.timestamp = 123456;

  lcmToIgn(lcmSimpleCarState, &ignSimpleCarState);

  EXPECT_EQ(2.56, ignSimpleCarState.x());
  EXPECT_EQ(8.39, ignSimpleCarState.y());
  EXPECT_EQ(123, ignSimpleCarState.mutable_time()->sec());
  EXPECT_EQ(456000000, ignSimpleCarState.mutable_time()->nsec());
}

//////////////////////////////////////////////////
/// \brief Test that an LCM viewer2_comm message describing the visualizer's
/// tree viewer response was properly translated to a custom ignition message.
GTEST_TEST(Viewer2CommsTest, TestViewer2CommsTranslation) {
  robotlocomotion::viewer2_comms_t lcmViewer2Comms;
  ignition::msgs::Viewer2Comms ignViewer2Comms;

  lcmViewer2Comms.utime = 123456789;
  lcmViewer2Comms.format = "format_string";
  lcmViewer2Comms.format_version_major = 5;
  lcmViewer2Comms.format_version_minor = 2;
  lcmViewer2Comms.num_bytes = 6;
  lcmViewer2Comms.data.resize(lcmViewer2Comms.num_bytes);
  lcmViewer2Comms.data = {'e', 'a', 'c', 0x14, 0x1e, 0x5d};

  lcmToIgn(lcmViewer2Comms, &ignViewer2Comms);

  EXPECT_EQ(123, ignViewer2Comms.mutable_time()->sec());
  EXPECT_EQ(456789000, ignViewer2Comms.mutable_time()->nsec());
  EXPECT_EQ("format_string", ignViewer2Comms.format());
  EXPECT_EQ(5, ignViewer2Comms.format_version_major());
  EXPECT_EQ(2, ignViewer2Comms.format_version_minor());
  EXPECT_EQ("eac\x14\x1e\x5d", ignViewer2Comms.data());
}

}  // namespace backend
}  // namespace delphyne
