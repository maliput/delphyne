#include "bridge/lcm_to_ign_translation.hh"
#include <iostream>
#include <gtest/gtest.h>
#include <ignition/msgs.hh>

namespace delphyne {
namespace bridge {

//////////////////////////////////////////////////
/// \brief Test that an LCM draw message is properly
/// translated to an ignition Geometry message.
class ViewerDrawTest : public ::testing::Test {
 protected:
  drake::lcmt_viewer_draw draw_msg;
  ignition::msgs::PosesStamped ign_poses_stamped;
  int last_index;

  virtual void SetUp() override {
    draw_msg.timestamp = 123456;
    draw_msg.num_links = 2;
    draw_msg.link_name.resize(draw_msg.num_links);
    draw_msg.robot_num.resize(draw_msg.num_links);
    draw_msg.position.resize(draw_msg.num_links);
    draw_msg.quaternion.resize(draw_msg.num_links);

    draw_msg.link_name[0] = "robot_one_link_one";
    draw_msg.robot_num[0] = 1;
    draw_msg.position[0] = {1.0, 2.0, 3.0};
    draw_msg.quaternion[0] = {4.0, 3.0, 2.0, 1.0};

    draw_msg.link_name[1] = "robot_two_link_one";
    draw_msg.robot_num[1] = 2;
    draw_msg.position[1] = {4.0, 5.0, 6.0};
    draw_msg.quaternion[1] = {8.0, 7.0, 6.0, 5.0};
    last_index = draw_msg.num_links - 1;
  }

  //////////////////////////////////////////////////
  /// \brief Checks that all the array-iterable values from
  /// lcmt_viewer_draw are matching their ignition counterpart
  void checkMsgTranslation(drake::lcmt_viewer_draw* lcm_msg,
                           ignition::msgs::PosesStamped* ign_poses) {
    for (int i = 0; i < lcm_msg->num_links; i++) {
      ASSERT_EQ(ign_poses->pose(i).name(), lcm_msg->link_name[i]);
      ASSERT_EQ(ign_poses->pose(i).id(), lcm_msg->robot_num[i]);
      ASSERT_EQ(ign_poses->pose(i).position().x(), lcm_msg->position[i][0]);
      ASSERT_EQ(ign_poses->pose(i).position().y(), lcm_msg->position[i][1]);
      ASSERT_EQ(ign_poses->pose(i).position().z(), lcm_msg->position[i][2]);
      ASSERT_EQ(ign_poses->pose(i).orientation().x(),
                lcm_msg->quaternion[i][0]);
      ASSERT_EQ(ign_poses->pose(i).orientation().y(),
                lcm_msg->quaternion[i][1]);
      ASSERT_EQ(ign_poses->pose(i).orientation().z(),
                lcm_msg->quaternion[i][2]);
      ASSERT_EQ(ign_poses->pose(i).orientation().w(),
                lcm_msg->quaternion[i][3]);
    }
  }
};

//////////////////////////////////////////////////
/// \brief Check that the translation of a PosesStamp
/// message with zero poses behaves as expected
TEST_F(ViewerDrawTest, TestZeroPosesInPosesStamp) {
  // Resize draw_msg defined in SetUp to zero poses
  draw_msg.num_links = 0;
  draw_msg.link_name.resize(0);
  draw_msg.robot_num.resize(0);
  draw_msg.position.resize(0);
  draw_msg.quaternion.resize(0);
  ignition::msgs::PosesStamped ign_msg;
  translate(draw_msg, &ign_msg);
  ASSERT_EQ(ign_msg.pose_size(), 0);

}

//////////////////////////////////////////////////
/// \brief Check that the translation of a PosesStamp
/// message with only one pose behaves as expected
TEST_F(ViewerDrawTest, TestOnePoseInPosesStamp) {
  // Resize draw_msg defined in SetUp to single pose
  draw_msg.num_links = 1;
  draw_msg.link_name.resize(1);
  draw_msg.robot_num.resize(1);
  draw_msg.position.resize(1);
  draw_msg.quaternion.resize(1);
  ignition::msgs::PosesStamped ign_msg;
  translate(draw_msg, &ign_msg);
  checkMsgTranslation(&draw_msg, &ign_msg);
}

//////////////////////////////////////////////////
/// \brief Check that the translation of a PosesStamp message
/// with more than one pose behaves as expected
TEST_F(ViewerDrawTest, TestThreePosesInPosesStamp) {
  // Resize draw_msg defined in SetUp to three poses
  draw_msg.num_links = 3;
  draw_msg.link_name.resize(3);
  draw_msg.robot_num.resize(3);
  draw_msg.position.resize(3);
  draw_msg.quaternion.resize(3);
  // Fill third link values
  draw_msg.link_name[2] = "robot_three_link_one";
  draw_msg.robot_num[2] = 3;
  draw_msg.position[2] = {7.0, 8.0, 9.0};
  draw_msg.quaternion[2] = {12.0, 11.0, 10.0, 9.0};
  ignition::msgs::PosesStamped ign_msg;
  translate(draw_msg, &ign_msg);
  checkMsgTranslation(&draw_msg, &ign_msg);
}

//////////////////////////////////////////////////
/// \brief Test that a header's sec and nsec were
/// correctly calculated
TEST_F(ViewerDrawTest, TestTimeStamp) {
  translate(draw_msg, &ign_poses_stamped);
  int secs = 123;
  int nsecs = 456000000;
  ASSERT_EQ(secs, ign_poses_stamped.time().sec());
  ASSERT_EQ(nsecs, ign_poses_stamped.time().nsec());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if at least one of the
///  defined position vectors doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionInPosition) {
  draw_msg.position[1] = {1.0, 2.0, 3.0, 4.0};
  EXPECT_THROW(translate(draw_msg, &ign_poses_stamped), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if at least
/// one of the defined orientation vectors doesn't
/// have the correct size
TEST_F(ViewerDrawTest, TestExceptionInOrientation) {
  draw_msg.quaternion[1] = {4.0, 2.0, 1.0};
  EXPECT_THROW(translate(draw_msg, &ign_poses_stamped), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// link_name vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfLinkNames) {
  draw_msg.link_name.resize(draw_msg.num_links + 1);
  EXPECT_THROW(translate(draw_msg, &ign_poses_stamped), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// robot_num vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfRobotNum) {
  draw_msg.robot_num.resize(draw_msg.num_links - 1);
  EXPECT_THROW(translate(draw_msg, &ign_poses_stamped), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// position vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfPositions) {
  draw_msg.position.resize(draw_msg.num_links - 1);
  EXPECT_THROW(translate(draw_msg, &ign_poses_stamped), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if the
/// quaternion vector doesn't have the correct size
TEST_F(ViewerDrawTest, TestExceptionNumberOfQuaternions) {
  draw_msg.position.resize(draw_msg.num_links + 1);
  EXPECT_THROW(translate(draw_msg, &ign_poses_stamped), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a
/// box is properly translated to an ignition Geometry message.
GTEST_TEST(BoxTest, TestBoxTranslation) {
  // Define LCM box geometry message
  drake::lcmt_viewer_geometry_data boxMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_box_geometry;
  // Fill LCM data
  boxMsg.type = boxMsg.BOX;
  boxMsg.num_float_data = 3;
  boxMsg.float_data.resize(boxMsg.num_float_data);
  boxMsg.float_data[0] = 3.888;
  boxMsg.float_data[1] = 0.753;
  boxMsg.float_data[2] = 0.031;

  // Translate from LCM to ignition
  translate(boxMsg, &ign_box_geometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ign_box_geometry.has_box());
  ASSERT_EQ(ignition::msgs::Geometry::BOX, ign_box_geometry.type());
  ASSERT_EQ(boxMsg.float_data[0], ign_box_geometry.box().size().x());
  ASSERT_EQ(boxMsg.float_data[1], ign_box_geometry.box().size().y());
  ASSERT_EQ(boxMsg.float_data[2], ign_box_geometry.box().size().z());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if an LCM geometry
/// message describing a box was not properly filled.
GTEST_TEST(BoxTest, TestExceptionInBoxTranslation) {
  // Define LCM box geometry message
  drake::lcmt_viewer_geometry_data boxMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_box_geometry;
  // Fill LCM data, but with missing z information
  boxMsg.type = boxMsg.BOX;
  boxMsg.num_float_data = 2;
  boxMsg.float_data.resize(boxMsg.num_float_data);
  boxMsg.float_data[0] = 1;
  boxMsg.float_data[1] = 2;

  EXPECT_THROW(translate(boxMsg, &ign_box_geometry), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a cylinder
/// was properly translated to an ignition Geometry message.
GTEST_TEST(CylinderTest, TestCylinderTranslation) {
  // Define LCM cylinder geometry message
  drake::lcmt_viewer_geometry_data cylinderMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_cylinder_geometry;
  // Fill LCM data
  cylinderMsg.type = cylinderMsg.CYLINDER;
  cylinderMsg.num_float_data = 2;
  cylinderMsg.float_data.resize(cylinderMsg.num_float_data);
  cylinderMsg.float_data[0] = 1.368;  // radius
  cylinderMsg.float_data[1] = 4.246;  // length

  // Translate from LCM to ignition
  translate(cylinderMsg, &ign_cylinder_geometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ign_cylinder_geometry.has_cylinder());
  ASSERT_EQ(ignition::msgs::Geometry::CYLINDER, ign_cylinder_geometry.type());
  ASSERT_EQ(cylinderMsg.float_data[0],
            ign_cylinder_geometry.cylinder().radius());
  ASSERT_EQ(cylinderMsg.float_data[1],
            ign_cylinder_geometry.cylinder().length());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if an LCM geometry
/// message describing a cylinder was not properly filled.
GTEST_TEST(CylinderTest, TestExceptionInCylinderTranslation) {
  // Define LCM cylinder geometry message
  drake::lcmt_viewer_geometry_data cylinderMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_cylinder_geometry;
  // Fill LCM data
  cylinderMsg.type = cylinderMsg.CYLINDER;
  cylinderMsg.num_float_data = 1;
  cylinderMsg.float_data.resize(cylinderMsg.num_float_data);
  cylinderMsg.float_data[0] = 1.368;  // radius

  EXPECT_THROW(translate(cylinderMsg, &ign_cylinder_geometry),
               TranslateException);
}

//////////////////////////////////////////////////
// \brief Test that an LCM geometry message describing a sphere
// was properly translated to an ignition Geometry message.
GTEST_TEST(SphereTest, TestSphereTranslation) {
  // Define LCM sphere geometry message
  drake::lcmt_viewer_geometry_data sphereMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_sphere_geometry;
  // Fill LCM data
  sphereMsg.type = sphereMsg.SPHERE;
  sphereMsg.num_float_data = 1;
  sphereMsg.float_data.resize(sphereMsg.num_float_data);
  sphereMsg.float_data[0] = 2.534;  // radius

  // Translate from LCM to ignition
  translate(sphereMsg, &ign_sphere_geometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ign_sphere_geometry.has_sphere());
  ASSERT_EQ(ignition::msgs::Geometry::SPHERE, ign_sphere_geometry.type());
  ASSERT_EQ(sphereMsg.float_data[0], ign_sphere_geometry.sphere().radius());
}

//////////////////////////////////////////////////
/// \brief Test that translation fails if an LCM geometry
/// message describing a sphere was not properly filled.
GTEST_TEST(SphereTest, TestExceptionInCylinderTranslation) {
  // Define LCM sphere geometry message
  drake::lcmt_viewer_geometry_data sphereMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_sphere_geometry;
  // Fill LCM data
  sphereMsg.type = sphereMsg.SPHERE;
  sphereMsg.num_float_data = 0;
  sphereMsg.float_data.resize(sphereMsg.num_float_data);

  EXPECT_THROW(translate(sphereMsg, &ign_sphere_geometry), TranslateException);
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a mesh with
/// scaling data was properly translated to an ignition Geometry message.
GTEST_TEST(MeshTest, TestMeshTranslationWithScale) {
  // Define LCM mesh geometry message
  drake::lcmt_viewer_geometry_data meshMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_mesh_geometry;
  // Fill LCM data
  meshMsg.type = meshMsg.MESH;
  meshMsg.string_data = "/path/to/mesh.obj";
  meshMsg.num_float_data = 3;
  meshMsg.float_data.resize(meshMsg.num_float_data);
  meshMsg.float_data[0] = 3.4;  // scale
  meshMsg.float_data[1] = 4.2;
  meshMsg.float_data[2] = 6.7;

  // Translate from LCM to ignition
  translate(meshMsg, &ign_mesh_geometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ign_mesh_geometry.has_mesh());
  ASSERT_EQ(ignition::msgs::Geometry::MESH, ign_mesh_geometry.type());
  ASSERT_EQ(meshMsg.string_data, ign_mesh_geometry.mesh().filename());
  ASSERT_TRUE(ign_mesh_geometry.mesh().has_scale());
  ASSERT_EQ(meshMsg.float_data[0], ign_mesh_geometry.mesh().scale().x());
  ASSERT_EQ(meshMsg.float_data[1], ign_mesh_geometry.mesh().scale().y());
  ASSERT_EQ(meshMsg.float_data[2], ign_mesh_geometry.mesh().scale().z());
}

//////////////////////////////////////////////////
/// \brief Test that an LCM geometry message describing a mesh without
/// scaling data is properly translated to an ignition Geometry message.
GTEST_TEST(MeshTest, TestMeshTranslationWithoutScale) {
  // Define LCM mesh geometry message
  drake::lcmt_viewer_geometry_data meshMsg;
  // Define the expected ignition geometry
  ignition::msgs::Geometry ign_mesh_geometry;
  // Fill LCM data
  meshMsg.type = meshMsg.MESH;
  meshMsg.string_data = "/path/to/mesh.obj";
  meshMsg.num_float_data = 0;
  meshMsg.float_data.resize(meshMsg.num_float_data);

  // Translate from LCM to ignition
  translate(meshMsg, &ign_mesh_geometry);

  // Verify generated ignition geometry
  ASSERT_EQ(true, ign_mesh_geometry.has_mesh());
  ASSERT_EQ(ignition::msgs::Geometry::MESH, ign_mesh_geometry.type());
  ASSERT_EQ(meshMsg.string_data, ign_mesh_geometry.mesh().filename());
  ASSERT_FALSE(ign_mesh_geometry.mesh().has_scale());
}

}  // namespace bridge
}  // namespace delphyne
