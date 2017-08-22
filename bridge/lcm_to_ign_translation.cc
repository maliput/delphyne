// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <ignition/msgs.hh>

#include "lcm_to_ign_translation.hh"

namespace delphyne {
namespace bridge {

void translateBoxGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                          ignition::msgs::Geometry* geometry_model);

void translateSphereGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                             ignition::msgs::Geometry* geometry_model);

void translateCylinderGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                               ignition::msgs::Geometry* geometry_model);

void translateMeshGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                           ignition::msgs::Geometry* geometry_model);

void checkVectorSize(int vector_size, int expected_size, std::string field_name);

//////////////////////////////////////////////////
void translate(drake::lcmt_viewer_draw robot_viewer_data,
               ignition::msgs::PosesStamped* poses_stamped_model) {
  // Check the size of each vector on an lcm_viewer_draw message
  // num_links represents the ammount of links declarated and
  // should be matched by the size of each of the following vectors
  checkVectorSize(robot_viewer_data.link_name.size(),
                  robot_viewer_data.num_links, "link_name");
  checkVectorSize(robot_viewer_data.robot_num.size(),
                  robot_viewer_data.num_links, "robot_num");
  checkVectorSize(robot_viewer_data.position.size(),
                  robot_viewer_data.num_links, "position");
  checkVectorSize(robot_viewer_data.quaternion.size(),
                  robot_viewer_data.num_links, "quaternion");

  // Convert from milliseconds to seconds
  int64_t sec = robot_viewer_data.timestamp / 1000;
  // Convert the remainder of division above to nanoseconds
  int64_t nsec = robot_viewer_data.timestamp % 1000 * 1000000;
  // Set timestamp
  poses_stamped_model->mutable_time()->set_sec(sec);
  poses_stamped_model->mutable_time()->set_nsec(nsec);

  // Add one pose per link
  for (int i = 0; i < robot_viewer_data.num_links; ++i) {
    ignition::msgs::Pose* pose_i = poses_stamped_model->add_pose();

    pose_i->set_name(robot_viewer_data.link_name[i]);
    pose_i->set_id(robot_viewer_data.robot_num[i]);
    // Check position size and translate
    checkVectorSize(robot_viewer_data.position[i].size(), 3,
                    "position[" + std::to_string(i) + "]");
    translate(robot_viewer_data.position[i].data(), pose_i->mutable_position());
    // Check orientation size and translate
    checkVectorSize(robot_viewer_data.quaternion[i].size(), 4,
                    "quaternion[" + std::to_string(i) + "]");
    translate(robot_viewer_data.quaternion[i].data(),
              pose_i->mutable_orientation());
  }
}

//////////////////////////////////////////////////
void translate(drake::lcmt_viewer_load_robot robot_data,
               ignition::msgs::Model* robot_model) {
  for (int i = 0; i < robot_data.num_links; ++i) {
    translate(robot_data.link[i], robot_model->add_link());
  }
}

//////////////////////////////////////////////////
void translate(drake::lcmt_viewer_link_data link_data,
               ignition::msgs::Link* link_model) {
  link_model->set_name(link_data.name);
  for (int i = 0; i < link_data.num_geom; ++i) {
    translate(link_data.geom[i], link_model->add_visual());
  }
}

//////////////////////////////////////////////////
void translate(drake::lcmt_viewer_geometry_data geometry_data,
               ignition::msgs::Visual* visual_model) {
  auto* pose_msg = visual_model->mutable_pose();
  auto* material_msg = visual_model->mutable_material();

  translate(geometry_data.position, pose_msg->mutable_position());
  translate(geometry_data.quaternion, pose_msg->mutable_orientation());
  translate(geometry_data.color, material_msg->mutable_diffuse());
  translate(geometry_data, visual_model->mutable_geometry());
}

//////////////////////////////////////////////////
void translate(float position_data[3],
               ignition::msgs::Vector3d* position_model) {
  position_model->set_x(position_data[0]);
  position_model->set_y(position_data[1]);
  position_model->set_z(position_data[2]);
}

//////////////////////////////////////////////////
void translate(float quaternion_data[4],
               ignition::msgs::Quaternion* quaternion_model) {
  quaternion_model->set_x(quaternion_data[0]);
  quaternion_model->set_y(quaternion_data[1]);
  quaternion_model->set_z(quaternion_data[2]);
  quaternion_model->set_w(quaternion_data[3]);
}

//////////////////////////////////////////////////
void translate(float color_data[4], ignition::msgs::Color* color_model) {
  color_model->set_r(color_data[0]);
  color_model->set_g(color_data[1]);
  color_model->set_b(color_data[2]);
  color_model->set_a(color_data[3]);
}

//////////////////////////////////////////////////
void translate(drake::lcmt_viewer_geometry_data geometry_data,
               ignition::msgs::Geometry* geometry_model) {
  switch (geometry_data.type) {
    case geometry_data.BOX:
      translateBoxGeometry(geometry_data, geometry_model);
      break;
    case geometry_data.SPHERE:
      translateSphereGeometry(geometry_data, geometry_model);
      break;
    case geometry_data.CYLINDER:
      translateCylinderGeometry(geometry_data, geometry_model);
      break;
    case geometry_data.MESH:
      translateMeshGeometry(geometry_data, geometry_model);
      break;
    default:
      std::map<int, std::string> unsupported_geometries;
      unsupported_geometries[geometry_data.CAPSULE] = "CAPSULE";
      unsupported_geometries[geometry_data.ELLIPSOID] = "ELLIPSOID";
      std::string type = std::to_string(geometry_data.type);
      if (unsupported_geometries.find(geometry_data.type) !=
          unsupported_geometries.end()) {
        type = unsupported_geometries[geometry_data.type];
      }
      throw TranslateException("Geometry of type: " + type +
                               " is currently unsupported");
      break;
  }
}

//////////////////////////////////////////////////
void translateBoxGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                          ignition::msgs::Geometry* geometry_model) {
  if (geometry_data.num_float_data != 3) {
    std::stringstream message;
    message << "Wrong float_data information for box: expecting 3 elements but "
            << geometry_data.num_float_data << " given.";
    throw TranslateException(message.str());
  }

  auto* box_msg = geometry_model->mutable_box();
  auto* size_msg = box_msg->mutable_size();

  geometry_model->set_type(ignition::msgs::Geometry::BOX);
  size_msg->set_x(geometry_data.float_data[0]);
  size_msg->set_y(geometry_data.float_data[1]);
  size_msg->set_z(geometry_data.float_data[2]);
}

//////////////////////////////////////////////////
void translateSphereGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                             ignition::msgs::Geometry* geometry_model) {
  if (geometry_data.num_float_data != 1) {
    std::stringstream message;
    message << "Wrong float_data information for sphere: "
            << "expecting 1 element but " << geometry_data.num_float_data
            << " given.";
    throw TranslateException(message.str());
  }

  auto* sphere_msg = geometry_model->mutable_sphere();

  geometry_model->set_type(ignition::msgs::Geometry::SPHERE);
  sphere_msg->set_radius(geometry_data.float_data[0]);
}

//////////////////////////////////////////////////
void translateCylinderGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                               ignition::msgs::Geometry* geometry_model) {
  if (geometry_data.num_float_data != 2) {
    std::stringstream message;
    message << "Wrong float_data information for cylinder: "
            << "expecting 2 elements but " << geometry_data.num_float_data
            << " given.";
    throw TranslateException(message.str());
  }
  auto* cylinder_msg = geometry_model->mutable_cylinder();

  geometry_model->set_type(ignition::msgs::Geometry::CYLINDER);
  cylinder_msg->set_radius(geometry_data.float_data[0]);
  cylinder_msg->set_length(geometry_data.float_data[1]);
}

//////////////////////////////////////////////////
void translateMeshGeometry(drake::lcmt_viewer_geometry_data geometry_data,
                           ignition::msgs::Geometry* geometry_model) {
  // If no string_data, we assume MeshMessage, which is unsupported
  if (geometry_data.string_data.empty()) {
    throw TranslateException(
        "Meshes generated from array are currently unsupported");
  } else {
    auto* mesh_msg = geometry_model->mutable_mesh();

    geometry_model->set_type(ignition::msgs::Geometry::MESH);
    mesh_msg->set_filename(geometry_data.string_data);

    if (geometry_data.num_float_data == 3) {
      auto* scale_msg = mesh_msg->mutable_scale();
      scale_msg->set_x(geometry_data.float_data[0]);
      scale_msg->set_y(geometry_data.float_data[1]);
      scale_msg->set_z(geometry_data.float_data[2]);
    }
  }
}

void checkVectorSize(int vector_size, int expected_size,
                     std::string field_name) {
  if (vector_size != expected_size) {
    std::stringstream message;
    message << "Wrong size for " << field_name << ": expecting "
            << expected_size << " elements but " << vector_size << " given.";
    throw TranslateException(message.str());
  }
}

}  // namespace bridge
}  // namespace delphyne
