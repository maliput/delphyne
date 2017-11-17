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
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <ignition/msgs.hh>

#include "bridge/lcm_to_ign_translation.h"
#include "bridge/translate_exception.h"

namespace delphyne {
namespace bridge {

void translateBoxGeometry(const drake::lcmt_viewer_geometry_data& geometryData,
                          ignition::msgs::Geometry* geometryModel);

void translateSphereGeometry(
    const drake::lcmt_viewer_geometry_data& geometryData,
    ignition::msgs::Geometry* geometryModel);

void translateCylinderGeometry(
    const drake::lcmt_viewer_geometry_data& geometryData,
    ignition::msgs::Geometry* geometryModel);

void translateMeshGeometry(const drake::lcmt_viewer_geometry_data& geometryData,
                           ignition::msgs::Geometry* geometryModel);

void checkVectorSize(int vectorSize, int expectedSize, std::string fieldName);

int64_t secsFromMillis(int64_t millis) { return millis / 1000; }

int64_t nsecsFromMillis(int64_t millis) { return millis % 1000 * 1000000; }

int64_t secsFromMicros(int64_t micros) { return micros / 1000000; }

int64_t nsecsFromMicros(int64_t micros) { return micros % 1000000 * 1000; }

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_simple_car_state_t& lcmData,
              ignition::msgs::SimpleCarState* ignData) {
  ignData->set_x(lcmData.x);
  ignData->set_y(lcmData.y);
  ignData->mutable_time()->set_sec(secsFromMillis(lcmData.timestamp));
  ignData->mutable_time()->set_nsec(nsecsFromMillis(lcmData.timestamp));
  ignData->set_heading(lcmData.heading);
  ignData->set_velocity(lcmData.velocity);
}

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_viewer_command& lcmData,
              ignition::msgs::ViewerCommand* ignData) {
  ignData->set_command_type(lcmData.command_type);
  ignData->set_command_data(lcmData.command_data);
}

//////////////////////////////////////////////////
void lcmToIgn(const robotlocomotion::viewer2_comms_t& lcmViewer2Data,
              ignition::msgs::Viewer2Comms* ignViewer2Data) {
  ignViewer2Data->mutable_time()->set_sec(secsFromMicros(lcmViewer2Data.utime));
  ignViewer2Data->mutable_time()->set_nsec(
      nsecsFromMicros(lcmViewer2Data.utime));
  ignViewer2Data->set_format(lcmViewer2Data.format);
  ignViewer2Data->set_format_version_major(lcmViewer2Data.format_version_major);
  ignViewer2Data->set_format_version_minor(lcmViewer2Data.format_version_minor);
  std::string lcmData(lcmViewer2Data.data.begin(), lcmViewer2Data.data.end());
  ignViewer2Data->set_data(lcmData);
}

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_viewer_draw& robotDrawData,
              ignition::msgs::Model_V* robotModels) {
  // Check the size of each vector on an lcm_viewer_draw message
  // num_links represents the amount of links declared and
  // should be matched by the size of each of the following vectors
  checkVectorSize(robotDrawData.link_name.size(), robotDrawData.num_links,
                  "link_name");
  checkVectorSize(robotDrawData.robot_num.size(), robotDrawData.num_links,
                  "robot_num");
  checkVectorSize(robotDrawData.position.size(), robotDrawData.num_links,
                  "position");
  checkVectorSize(robotDrawData.quaternion.size(), robotDrawData.num_links,
                  "quaternion");

  // Convert from milliseconds to seconds
  int64_t sec = secsFromMillis(robotDrawData.timestamp);
  // Convert the remainder of division above to nanoseconds
  int64_t nsec = nsecsFromMillis(robotDrawData.timestamp);

  robotModels->mutable_header()->mutable_stamp()->set_sec(sec);
  robotModels->mutable_header()->mutable_stamp()->set_nsec(nsec);

  std::map<int32_t, ignition::msgs::Model*> models;

  // Add one pose per link
  for (int i = 0; i < robotDrawData.num_links; ++i) {
    int32_t robotId = robotDrawData.robot_num[i];
    if (models.count(robotId) == 0) {
      models[robotId] = robotModels->add_models();
      models[robotId]->set_id(robotId);
    }

    ignition::msgs::Model* robotModel = models[robotId];
    ignition::msgs::Link* link = robotModel->add_link();
    ignition::msgs::Pose* pose = link->mutable_pose();

    link->set_name(robotDrawData.link_name[i]);

    // Check position size and translate
    checkVectorSize(robotDrawData.position[i].size(), 3,
                    "position[" + std::to_string(i) + "]");
    lcmToIgn(robotDrawData.position[i].data(), pose->mutable_position());

    // Check orientation size and translate
    checkVectorSize(robotDrawData.quaternion[i].size(), 4,
                    "quaternion[" + std::to_string(i) + "]");
    lcmToIgn(robotDrawData.quaternion[i].data(), pose->mutable_orientation());
  }
}

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_viewer_load_robot& robotData,
              ignition::msgs::Model_V* robotModels) {
  std::map<int32_t, std::vector<drake::lcmt_viewer_link_data>> groupedLinks;

  for (int i = 0; i < robotData.num_links; ++i) {
    int32_t robotId = robotData.link[i].robot_num;
    if (groupedLinks.count(robotId) == 0) {
      groupedLinks[robotId] = {};
    }
    groupedLinks[robotId].push_back(robotData.link[i]);
  }

  for (auto iterator = groupedLinks.begin(); iterator != groupedLinks.end();
       ++iterator) {
    drake::lcmt_viewer_load_robot robot;
    auto links = iterator->second;

    robot.num_links = links.size();
    robot.link.resize(robot.num_links);
    for (int j = 0; j < robot.num_links; ++j) {
      robot.link[j] = links[j];
    }

    lcmToIgn(robot, robotModels->add_models());
  }
}

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_viewer_load_robot& robotData,
              ignition::msgs::Model* robotModel) {
  for (int i = 0; i < robotData.num_links; ++i) {
    lcmToIgn(robotData.link[i], robotModel->add_link());
  }
  // If there is at least one link, take the robot id from it
  if (robotData.num_links > 0) {
    robotModel->set_id(robotData.link[0].robot_num);
  }
}

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_viewer_link_data& linkData,
              ignition::msgs::Link* linkModel) {
  linkModel->set_name(linkData.name);
  for (int i = 0; i < linkData.num_geom; ++i) {
    lcmToIgn(linkData.geom[i], linkModel->add_visual());
  }
}

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_viewer_geometry_data& geometryData,
              ignition::msgs::Visual* visualModel) {
  auto* poseMsg = visualModel->mutable_pose();
  auto* materialMsg = visualModel->mutable_material();

  lcmToIgn(geometryData.position, poseMsg->mutable_position());
  lcmToIgn(geometryData.quaternion, poseMsg->mutable_orientation());
  lcmToIgn(geometryData.color, materialMsg->mutable_diffuse());
  lcmToIgn(geometryData, visualModel->mutable_geometry());
}

//////////////////////////////////////////////////
void lcmToIgn(const float positionData[3],
              ignition::msgs::Vector3d* positionModel) {
  positionModel->set_x(positionData[0]);
  positionModel->set_y(positionData[1]);
  positionModel->set_z(positionData[2]);
}

//////////////////////////////////////////////////
void lcmToIgn(const float quaternionData[4],
              ignition::msgs::Quaternion* quaternionModel) {
  quaternionModel->set_w(quaternionData[0]);
  quaternionModel->set_x(quaternionData[1]);
  quaternionModel->set_y(quaternionData[2]);
  quaternionModel->set_z(quaternionData[3]);
}

//////////////////////////////////////////////////
void lcmToIgn(const float colorData[4], ignition::msgs::Color* colorModel) {
  colorModel->set_r(colorData[0]);
  colorModel->set_g(colorData[1]);
  colorModel->set_b(colorData[2]);
  colorModel->set_a(colorData[3]);
}

//////////////////////////////////////////////////
void lcmToIgn(const drake::lcmt_viewer_geometry_data& geometryData,
              ignition::msgs::Geometry* geometryModel) {
  switch (geometryData.type) {
    case drake::lcmt_viewer_geometry_data::BOX:
      translateBoxGeometry(geometryData, geometryModel);
      break;
    case drake::lcmt_viewer_geometry_data::SPHERE:
      translateSphereGeometry(geometryData, geometryModel);
      break;
    case drake::lcmt_viewer_geometry_data::CYLINDER:
      translateCylinderGeometry(geometryData, geometryModel);
      break;
    case drake::lcmt_viewer_geometry_data::MESH:
      translateMeshGeometry(geometryData, geometryModel);
      break;
    default:
      std::map<int, std::string> unsupportedGeometries;
      unsupportedGeometries[geometryData.CAPSULE] = "CAPSULE";
      unsupportedGeometries[geometryData.ELLIPSOID] = "ELLIPSOID";
      std::string type = std::to_string(geometryData.type);
      if (unsupportedGeometries.find(geometryData.type) !=
          unsupportedGeometries.end()) {
        type = unsupportedGeometries[geometryData.type];
      }
      throw TranslateException("Geometry of type: " + type +
                               " is currently unsupported");
      break;
  }
}

//////////////////////////////////////////////////
void translateBoxGeometry(const drake::lcmt_viewer_geometry_data& geometryData,
                          ignition::msgs::Geometry* geometryModel) {
  if (geometryData.num_float_data != 3) {
    std::stringstream message;
    message << "Wrong float_data information for box: expecting 3 elements but "
            << geometryData.num_float_data << " given.";
    throw TranslateException(message.str());
  }

  auto* boxMsg = geometryModel->mutable_box();
  auto* sizeMsg = boxMsg->mutable_size();

  geometryModel->set_type(ignition::msgs::Geometry::BOX);
  sizeMsg->set_x(geometryData.float_data[0]);
  sizeMsg->set_y(geometryData.float_data[1]);
  sizeMsg->set_z(geometryData.float_data[2]);
}

//////////////////////////////////////////////////
void translateSphereGeometry(
    const drake::lcmt_viewer_geometry_data& geometryData,
    ignition::msgs::Geometry* geometryModel) {
  if (geometryData.num_float_data != 1) {
    std::stringstream message;
    message << "Wrong float_data information for sphere: "
            << "expecting 1 element but " << geometryData.num_float_data
            << " given.";
    throw TranslateException(message.str());
  }

  auto* sphereMsg = geometryModel->mutable_sphere();

  geometryModel->set_type(ignition::msgs::Geometry::SPHERE);
  sphereMsg->set_radius(geometryData.float_data[0]);
}

//////////////////////////////////////////////////
void translateCylinderGeometry(
    const drake::lcmt_viewer_geometry_data& geometryData,
    ignition::msgs::Geometry* geometryModel) {
  if (geometryData.num_float_data != 2) {
    std::stringstream message;
    message << "Wrong float_data information for cylinder: "
            << "expecting 2 elements but " << geometryData.num_float_data
            << " given.";
    throw TranslateException(message.str());
  }
  auto* cylinderMsg = geometryModel->mutable_cylinder();

  geometryModel->set_type(ignition::msgs::Geometry::CYLINDER);
  cylinderMsg->set_radius(geometryData.float_data[0]);
  cylinderMsg->set_length(geometryData.float_data[1]);
}

//////////////////////////////////////////////////
void translateMeshGeometry(const drake::lcmt_viewer_geometry_data& geometryData,
                           ignition::msgs::Geometry* geometryModel) {
  // If no string_data, we assume MeshMessage, which is unsupported
  if (geometryData.string_data.empty()) {
    throw TranslateException(
        "Meshes generated from array are currently unsupported");
  } else {
    auto* meshMsh = geometryModel->mutable_mesh();

    geometryModel->set_type(ignition::msgs::Geometry::MESH);
    meshMsh->set_filename(geometryData.string_data);

    if (geometryData.num_float_data == 3) {
      auto* scale_msg = meshMsh->mutable_scale();
      scale_msg->set_x(geometryData.float_data[0]);
      scale_msg->set_y(geometryData.float_data[1]);
      scale_msg->set_z(geometryData.float_data[2]);
    }
  }
}

void checkVectorSize(int vectorSize, int expectedSize, std::string fieldName) {
  if (vectorSize != expectedSize) {
    std::stringstream message;
    message << "Wrong size for " << fieldName << ": expecting " << expectedSize
            << " elements but " << vectorSize << " given.";
    throw TranslateException(message.str());
  }
}

}  // namespace bridge
}  // namespace delphyne
