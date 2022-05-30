// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "backend/ign_models_traffic_lights.h"

#include <map>

#include <drake/common/eigen_types.h>
#include <drake/systems/rendering/pose_bundle.h>

namespace delphyne {

const maliput::math::Vector4 IgnModelsTrafficLights::kRedBulbColorOn{1.0, 0.0, 0.0, 1.0};
const maliput::math::Vector4 IgnModelsTrafficLights::kGreenBulbColorOn{0.0, 1.0, 0.0, 1.0};
const maliput::math::Vector4 IgnModelsTrafficLights::kYellowBulbColorOn{1.0, 1.0, 0.0, 1.0};
const maliput::math::Vector4 IgnModelsTrafficLights::kRedBulbColorOff{0.1, 0.0, 0.0, 1.0};
const maliput::math::Vector4 IgnModelsTrafficLights::kGreenBulbColorOff{0.0, 0.1, 0.0, 1.0};
const maliput::math::Vector4 IgnModelsTrafficLights::kYellowBulbColorOff{0.1, 0.1, 0.0, 1.0};

IgnModelsTrafficLights::IgnModelsTrafficLights(maliput::api::RoadNetwork* road_network) : road_network_(road_network) {
  models_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<ignition::msgs::Model_V>()).get_index();

  traffic_light_output_port_index_ =
      DeclareAbstractOutputPort(&IgnModelsTrafficLights::CalcIgnModelVMessage).get_index();
  new_data_output_port_index_ = DeclareAbstractOutputPort(&IgnModelsTrafficLights::CalcNewDataMessage).get_index();
}

void IgnModelsTrafficLights::CalcNewDataMessage(const drake::systems::Context<double>&,
                                                ignition::msgs::Boolean* output_new_data) const {
  output_new_data->set_data(new_data_);
  new_data_ = false;
}

void IgnModelsTrafficLights::CalcIgnModelVMessage(const drake::systems::Context<double>& context,
                                                  ignition::msgs::Model_V* output_models) const {
  // Ensures provided output models message is cleared.
  output_models->Clear();

  // Retrieves models and states inputs.
  const ignition::msgs::Model_V* input_models =
      this->template EvalInputValue<ignition::msgs::Model_V>(context, models_input_port_index_);

  // Copies timestamp from input to output.
  output_models->mutable_header()->mutable_stamp()->CopyFrom(input_models->header().stamp());

  // Gather all the current bulb states in the road network.
  maliput::api::rules::BulbStates all_bulb_states;
  for (const auto& intersection : road_network_->intersection_book()->GetIntersections()) {
    const std::optional<maliput::api::rules::BulbStates> bulb_states_opt = intersection->bulb_states();
    if (!bulb_states_opt.has_value()) {
      continue;
    }
    all_bulb_states.insert(bulb_states_opt.value().begin(), bulb_states_opt.value().end());
  }

  // Set the new data output to true as there are new states.
  if (all_bulb_states != last_bulb_states_) {
    new_data_ = true;
  }
  last_bulb_states_ = all_bulb_states;

  // Iterates over input models looking for models related to traffic lights.
  // These models are copied over to the output while updating their bulb states.
  for (const ignition::msgs::Model& input_model : input_models->models()) {
    if (std::find_if(input_model.link().begin(), input_model.link().end(), [&all_bulb_states](const auto& link) {
          return std::find_if(all_bulb_states.begin(), all_bulb_states.end(), [&link](const auto& bulb_state) {
                   return bulb_state.first.string() == link.name();
                 }) != all_bulb_states.end();
        }) != input_model.link().end()) {
      ignition::msgs::Model* output_model = output_models->add_models();
      output_model->CopyFrom(input_model);
      for (int i = 0; i < output_model->link().size(); ++i) {
        auto link = output_model->mutable_link(i);
        const std::string bulb_unique_id_str{link->name()};
        const auto bulb_state_itr = std::find_if(
            all_bulb_states.begin(), all_bulb_states.end(),
            [&bulb_unique_id_str](const auto& bulb_state) { return bulb_state.first.string() == bulb_unique_id_str; });
        const auto bulb_color = road_network_->traffic_light_book()
                                    ->GetTrafficLight(bulb_state_itr->first.traffic_light_id())
                                    ->GetBulbGroup(bulb_state_itr->first.bulb_group_id())
                                    ->GetBulb(bulb_state_itr->first.bulb_id())
                                    ->color();
        SetBulbState(link, bulb_color, all_bulb_states.at(bulb_state_itr->first));
      }
    }
  }
}

void IgnModelsTrafficLights::SetBulbState(ignition::msgs::Link* link, const maliput::api::rules::BulbColor& bulb_color,
                                          maliput::api::rules::BulbState& bulb_state) const {
  maliput::math::Vector4 color_to_set;
  switch (bulb_color) {
    case maliput::api::rules::BulbColor::kRed:
      color_to_set = bulb_state == maliput::api::rules::BulbState::kOn ? kRedBulbColorOn : kRedBulbColorOff;
      break;
    case maliput::api::rules::BulbColor::kGreen:
      color_to_set = bulb_state == maliput::api::rules::BulbState::kOn ? kGreenBulbColorOn : kGreenBulbColorOff;
      break;
    case maliput::api::rules::BulbColor::kYellow:
      color_to_set = bulb_state == maliput::api::rules::BulbState::kOn ? kYellowBulbColorOn : kYellowBulbColorOff;
      break;
    default:
      throw std::runtime_error("Unknown bulb color");
  }
  link->mutable_visual()->begin()->mutable_material()->mutable_diffuse()->set_r(color_to_set.x());
  link->mutable_visual()->begin()->mutable_material()->mutable_diffuse()->set_g(color_to_set.y());
  link->mutable_visual()->begin()->mutable_material()->mutable_diffuse()->set_b(color_to_set.z());
}

}  // namespace delphyne
