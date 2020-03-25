// Copyright 2019 Toyota Research Institute
#include "backend/geometry_utilities.h"

#include <algorithm>
#include <string>

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <maliput/utilities/generate_urdf.h>

#include "delphyne/macros.h"

namespace delphyne {

drake::lcmt_viewer_load_robot BuildLoadMessageForRoad(const maliput::api::RoadGeometry& road_geometry,
                                                      const maliput::utility::ObjFeatures& features) {
  drake::geometry::SceneGraph<double> scene_graph;
  drake::multibody::MultibodyPlant<double> plant(0.0);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  std::string filename = road_geometry.id().string();
  std::transform(filename.begin(), filename.end(), filename.begin(), [](char ch) { return ch == ' ' ? '_' : ch; });
  maliput::utility::GenerateUrdfFile(&road_geometry, "/tmp", filename, features);
  drake::multibody::Parser parser(&plant);
  parser.AddModelFromFile("/tmp/" + filename + ".urdf");
  plant.Finalize();
  drake::lcmt_viewer_load_robot load_message = BuildLoadMessage(scene_graph);
  for (drake::lcmt_viewer_link_data& link : load_message.link) {
    DELPHYNE_VALIDATE(link.geom.size() == 1, std::runtime_error, "Expected one geometry in link");
    link.name = link.name.substr(link.name.rfind("::") + 2);
    link.geom[0].string_data.append("?culling=off");
  }
  return load_message;
}

}  // namespace delphyne
