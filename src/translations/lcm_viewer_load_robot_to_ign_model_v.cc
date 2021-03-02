// Copyright 2018 Toyota Research Institute

#include "translations/lcm_viewer_load_robot_to_ign_model_v.h"

#include <map>
#include <vector>

// public headers
#include "delphyne/macros.h"

// private headers
#include "translations/generate_unique_id.h"
#include "translations/time_conversion.h"

namespace delphyne {

void LcmViewerLoadRobotToIgnModelV::DoDrakeToIgnTranslation(const drake::lcmt_viewer_load_robot& lcm_message,
                                                            ignition::msgs::Model_V* ign_message, int64_t time) const {
  DELPHYNE_VALIDATE(ign_message != nullptr, std::invalid_argument, "Ignition message pointer must not be null");

  // Clears state from the previous call.
  // @see DrakeToIgn::DoDrakeToIgnTranslation
  ign_message->Clear();

  ign_message->mutable_header()->mutable_stamp()->CopyFrom(MillisToIgnitionTime(time));

  // An lcmt_viewer_load_robot is a vector of links, some of which have the same
  // id, and correspond to the same model. To make translation into models
  // easier, a first pass over said vector is done, grouping by link id.
  std::map<int32_t, std::vector<const drake::lcmt_viewer_link_data*>> grouped_links_by_id;

  for (const drake::lcmt_viewer_link_data& link : lcm_message.link) {
    const int32_t id = link.robot_num;
    if (grouped_links_by_id.count(id) == 0) {
      grouped_links_by_id[id] = {};
    }
    grouped_links_by_id[id].push_back(&link);
  }

  // A model is created for each id, and all links with that same id are added
  // to it.
  for (const auto& id_links_pair : grouped_links_by_id) {
    ignition::msgs::Model* new_model = ign_message->add_models();
    new_model->set_id(id_links_pair.first);

    for (const auto& link : id_links_pair.second) {
      ignition::msgs::Link* new_link = new_model->add_link();
      new_link->set_name(link->name);

      // Add unique integer id per link
      const size_t link_id = GenerateLinkId(new_model->id(), link->name);
      new_link->set_id(link_id);

      for (size_t v = 0; v < link->geom.size(); ++v) {
        const drake::lcmt_viewer_geometry_data& geometry = link->geom[v];
        const size_t visual_id = GenerateVisualId(new_model->id(), link->name, v);

        // The ignition counterpart for an LCM geometry is an ignition visual,
        // which has different fields (geometry, pose, material, etc.) in which
        // the corresponding parts of the LCM geometry are stored.
        ignition::msgs::Visual* new_visual = new_link->add_visual();
        new_visual->set_id(visual_id);

        LcmGeometryToIgnition(geometry, new_visual->mutable_geometry());

        ignition::msgs::Pose* pose = new_visual->mutable_pose();
        pose->set_id(visual_id);
        PositionArrayToIgnition(geometry.position, pose->mutable_position());
        QuaternionArrayToIgnition(geometry.quaternion, pose->mutable_orientation());

        ignition::msgs::Material* material = new_visual->mutable_material();
        LcmColorToIgnition(geometry.color, material->mutable_diffuse());
      }
    }
  }
}

}  // namespace delphyne
