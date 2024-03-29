// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include "backend/agent_simulation_builder.h"

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <drake/common/eigen_types.h>
#include <drake/common/text_logging.h>
#include <drake/systems/analysis/runge_kutta2_integrator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <maliput/api/road_geometry.h>
#include <maliput/utility/generate_obj.h>

#include "backend/dynamic_environment_handler_system.h"
#include "backend/fixed_phase_iteration_handler.h"
#include "backend/geometry_utilities.h"
#include "backend/ign_commanded_pass_through.h"
#include "backend/ign_models_assembler.h"
#include "backend/ign_models_to_ids.h"
#include "backend/ign_models_traffic_lights.h"
#include "backend/ign_publisher_system.h"
#include "delphyne/macros.h"
#include "delphyne/protobuf/agent_state.pb.h"
#include "delphyne/protobuf/agent_state_v.pb.h"
#include "translations/agent_state_v_splitter.h"
#include "translations/ign_driving_command_to_drake.h"
#include "translations/ign_model_v_to_ign_pose_v.h"
#include "translations/lcm_viewer_draw_to_ign_model_v.h"
#include "translations/lcm_viewer_load_robot_to_ign_model_v.h"
#include "translations/pose_bundle_to_agent_state_v.h"

namespace delphyne {

using drake::AbstractValue;
using drake::systems::RungeKutta2Integrator;
using drake::systems::SystemOutput;
using drake::systems::rendering::PoseBundle;
using maliput::api::RoadGeometry;

template <typename T>
constexpr double AgentSimulationBaseBuilder<T>::kSceneTreePublishRateHz;

template <typename T>
constexpr const char* AgentSimulationBaseBuilder<T>::kSceneTreeTopicName;

template <typename T>
constexpr double AgentSimulationBaseBuilder<T>::kSceneUpdatesPublishRateHz;

template <typename T>
constexpr const char* AgentSimulationBaseBuilder<T>::kSceneUpdatesTopicName;

template <typename T>
constexpr const char* AgentSimulationBaseBuilder<T>::kAggregatedAgentsStateTopicName;

template <typename T>
AgentSimulationBaseBuilder<T>::AgentSimulationBaseBuilder() {
#ifdef HAVE_SPDLOG
  // Avoid the many & varied 'info' level logging messages coming from drake.
  //
  // Note: Drake will have defined HAVE_SPDLOG if it is using that
  // (see lib/cmake/spdlog/spdlog-config.cmake that was installed by drake).
  //
  // It would be preferable if drake did not spam downstream application
  // development on info channels, but that will require a discussion and
  // a large swipe at the many uses in drake.
  //
  // Now, also avoid 'warn' level logging messages coming from drake MBT.
  drake::log()->set_level(spdlog::level::err);
#endif
  Reset();
}

template <typename T>
void AgentSimulationBaseBuilder<T>::Reset() {
  // Resets Diagram builder.
  builder_ = std::make_unique<drake::systems::DiagramBuilder<T>>();

  // Resets agent ID sequence.
  agent_id_sequence_ = 0;

  // Adds pose aggregator system for agents' poses.
  aggregator_ = builder_->template AddSystem<drake::systems::rendering::PoseAggregator<T>>();
  aggregator_->set_name("pose_aggregator");
  builder_->ExportOutput(aggregator_->get_output_port(0));

  // Adds agents' visuals applicator system.
  car_vis_applicator_ = builder_->template AddSystem<CarVisApplicator<T>>();
  car_vis_applicator_->set_name("car_vis_applicator");
  builder_->Connect(aggregator_->get_output_port(0), car_vis_applicator_->get_car_poses_input_port());

  // Adds scene graph system for agents' collision geometries.
  scene_graph_ = builder_->template AddSystem<drake::geometry::SceneGraph<T>>();
  scene_graph_->set_name("scene_graph");
}

// TODO(hidmic): Figure out why this is necessary, as SceneGraph ID duplication
// ensues otherwise. There seems to be a (very) subtle issue when linking the
// template class that keeps the atomic sequence of FrameIds.
template <typename T>
void AgentSimulationBaseBuilder<T>::DoAddAgent(AgentBaseBlueprint<T>* blueprint) {
  // Builds and validates the agent.
  std::unique_ptr<AgentBase<T>> agent = blueprint->BuildInto(road_network_.get(), builder_.get());
  const int agent_id = agent_id_sequence_++;
  const std::string& agent_name = agent->name();
  DELPHYNE_VALIDATE(agents_.count(agent_name) == 0, std::runtime_error,
                    "An agent named \"" + agent_name + "\" already exists.");

  // Wires up the agent's ports.
  typename AgentBase<T>::Diagram* agent_diagram = blueprint->GetMutableDiagram(agent.get());
  drake::systems::rendering::PoseVelocityInputPorts<double> ports =
      aggregator_->AddSinglePoseAndVelocityInput(agent_name, agent_id);
  builder_->Connect(agent_diagram->get_output_port("pose"), ports.pose_input_port);
  builder_->Connect(agent_diagram->get_output_port("velocity"), ports.velocity_input_port);
  builder_->Connect(aggregator_->get_output_port(0), agent_diagram->get_input_port("traffic_poses"));

  // Registers and wires up a Prius geometry for both visuals and collision
  // geometries.

  // TODO(daniel.stonier) this just enforces ... 'everything is a Prius'.
  // We'll need a means of having the agents report what visual they have and
  // hooking that up. Also wondering why visuals are in the drake diagram?
  car_vis_applicator_->AddCarVis(std::make_unique<SimplePriusVis<T>>(agent_id, agent_name));

  builder_->Connect(
      agent_diagram->get_output_port("pose"),
      WirePriusGeometry(agent_name, builder_.get(), scene_graph_, blueprint->GetMutableGeometryIDs(agent.get())));

  agents_[agent_name] = std::move(agent);
}

namespace {

// Get default features of the road mesh.
maliput::utility::ObjFeatures GetDefaultFeatures() {
  maliput::utility::ObjFeatures features;
  // Max distance between rendered vertices (in s- or r-dimension), in meters.
  features.max_grid_unit = 1.0;
  // Min number of vertices (in s- or r-dimension).
  features.min_grid_resolution = 5.0;
  // Using the standard linear tolerance, which gives great results most of
  // the time.
  features.simplify_mesh_threshold = 0.01;
  features.draw_elevation_bounds = false;
  features.draw_stripes = true;
  features.draw_arrows = false;
  features.draw_lane_haze = false;
  features.draw_branch_points = false;
  return features;
}

}  // namespace

template <typename T>
const maliput::api::RoadGeometry* AgentSimulationBaseBuilder<T>::GetRoadGeometry() const {
  return road_network_ == nullptr ? nullptr : road_network_->road_geometry();
}

template <typename T>
const maliput::api::RoadNetwork* AgentSimulationBaseBuilder<T>::SetRoadNetwork(
    std::unique_ptr<maliput::api::RoadNetwork> road_network) {
  return SetRoadNetwork(std::move(road_network), GetDefaultFeatures());
}

template <typename T>
const maliput::api::RoadNetwork* AgentSimulationBaseBuilder<T>::SetRoadNetwork(
    std::unique_ptr<maliput::api::RoadNetwork> road_network, const maliput::utility::ObjFeatures& features) {
  DELPHYNE_DEMAND(road_network != nullptr);
  DELPHYNE_DEMAND(road_geometry_ == nullptr);
  road_network_ = std::move(road_network);
  road_features_ = features;
  return road_network_.get();
}

template <typename T>
SceneSystem* AgentSimulationBaseBuilder<T>::AddScenePublishers() {
  // Adds a translation system that takes the output of a CarVisApplicator
  // and creates an lcmt_viewer_draw message containing the latest poses of
  // the visual elements.
  auto bundle_to_draw = builder_->template AddSystem<drake::systems::rendering::PoseBundleToDrawMessage>();
  bundle_to_draw->set_name("bundle_to_draw");

  // The bundle of poses are translated into an LCM viewer draw message.
  builder_->Connect(car_vis_applicator_->get_visual_geometry_poses_output_port(), bundle_to_draw->get_input_port(0));

  // The LCM viewer draw message is translated into an ignition Model_V message.
  auto viewer_draw_translator = builder_->template AddSystem<LcmViewerDrawToIgnModelV>();
  builder_->Connect(*bundle_to_draw, *viewer_draw_translator);

  // The translated Model_V message is then published.
  auto model_v_publisher = builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::Model_V>>(
      kSceneUpdatesTopicName, kSceneUpdatesPublishRateHz);

  // The translated ignition message is then published.
  builder_->Connect(*viewer_draw_translator, *model_v_publisher);

  // The Model_V message is translated into an ignition Pose_V message to support visualizer2.
  auto pose_v_translator = builder_->template AddSystem<IgnModelVToIgnPoseV>();
  builder_->Connect(*viewer_draw_translator, *pose_v_translator);

  // The Pose_V message is then published.
  auto pose_v_publisher = builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::Pose_V>>(
      kPoseUpdatesTopicName, kSceneUpdatesPublishRateHz);
  builder_->Connect(*pose_v_translator, *pose_v_publisher);

  // An ignition scene message is built from the geometry description, and both
  // model and links' updated poses.

  // The geometry description is retrieved from multiple sources as LCM viewer
  // load robot messages, so those need to be aggregated.
  std::vector<drake::lcmt_viewer_load_robot> messages{car_vis_applicator_->get_load_robot_message()};
  const maliput::api::RoadGeometry* road_geometry =
      road_network_ != nullptr ? road_network_->road_geometry() : road_geometry_.get();
  if (road_geometry != nullptr) {
    messages.push_back(BuildLoadMessageForRoad(*road_geometry, road_features_));
  }
  // Adds traffic lights models.
  // First check whether traffic lights are present in the road network.
  const bool traffic_light_system = road_network_ != nullptr && road_network_->traffic_light_book() != nullptr &&
                                    !road_network_->traffic_light_book()->TrafficLights().empty();
  if (traffic_light_system) {
    messages.push_back(BuildLoadMessageForTrafficLights(road_network_->traffic_light_book()->TrafficLights()));
  }

  // Adds an aggregator system to aggregate multiple lcmt_viewer_load_robot
  // messages into a single one containing all models in the scene.
  auto load_robot_aggregator = builder_->template AddSystem<LoadRobotAggregator>(messages);

  // The aggregated LCM viewer load robot message containing the geometry
  // description is translated into an ignition Model_V message.
  auto viewer_load_robot_translator = builder_->template AddSystem<LcmViewerLoadRobotToIgnModelV>();
  builder_->Connect(*load_robot_aggregator, *viewer_load_robot_translator);

  auto scene_system = builder_->template AddSystem<SceneSystem>();
  scene_system->set_name("scene_system");

  // The Model_V describing the geometry is finally used to build the scene.
  builder_->Connect(viewer_load_robot_translator->get_output_port(0), scene_system->get_geometry_models_input_port());

  // Updated model and links poses are stored in the Model_V message that
  // is assembled from an LCM viewer draw translation and a pose bundle.
  auto models_assembler = builder_->template AddSystem<IgnModelsAssembler>();

  builder_->Connect(viewer_draw_translator->get_output_port(0), models_assembler->get_models_input_port());

  builder_->Connect(aggregator_->get_output_port(0), models_assembler->get_states_input_port());

  builder_->Connect(models_assembler->get_output_port(0), scene_system->get_updated_pose_models_input_port());

  if (traffic_light_system) {
    // Update traffic lights models according to the traffic light book states.
    auto models_traffic_lights = builder_->template AddSystem<IgnModelsTrafficLights>(road_network_.get());
    builder_->Connect(viewer_load_robot_translator->get_output_port(0), models_traffic_lights->get_models_input_port());
    builder_->Connect(models_traffic_lights->get_traffic_lights_models_output_port(),
                      scene_system->get_updated_visual_models_input_port());

    // We can't update just the material/color of the traffic light bulb, so we need to delete the model and re-add it
    // to the scene. Publishing the models to be deleted to the deletion_topic alerts the render engine to delete the
    // models. The scene message being published at scene_update topic will add the models with the new material back to
    // the scene.

    // Converts traffic lights models to ids.
    auto traffic_lights_models_to_ids = builder_->template AddSystem<IgnModelsToIds>();
    builder_->Connect(models_traffic_lights->get_traffic_lights_models_output_port(),
                      traffic_lights_models_to_ids->get_models_input_port());
    // We want to delete the traffic light models only when it changes its state.
    // The switch is used to pass through the ids only when the models should be deleted. And that time is sync using
    // the output port that indicates new data is available.
    auto pass_through_switch = builder_->template AddSystem<IgnCommandedPassThrough<ignition::msgs::UInt32_V>>();
    builder_->Connect(traffic_lights_models_to_ids->get_output_port(0), pass_through_switch->get_data_input_port());
    builder_->Connect(models_traffic_lights->get_new_data_output_port(), pass_through_switch->get_switch_input_port());
    // Connects the pass through switch to the publisher system to publish the ids to be deleted.
    auto traffic_light_ids_publisher = builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::UInt32_V>>(
        kSceneDeletionTopicName, kSceneUpdatesPublishRateHz);
    builder_->Connect(*pass_through_switch, *traffic_light_ids_publisher);
  }

  // The scene is then published over a scene topic to update the scene tree
  // widget of the visualizer. Because this information is not needed at the
  // same frequency the simulation runs at, the publishing frequency is reduced.
  auto scene_publisher = builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::Scene>>(
      kSceneTreeTopicName, kSceneTreePublishRateHz);
  builder_->Connect(*scene_system, *scene_publisher);

  return scene_system;
}

template <typename T>
void AgentSimulationBaseBuilder<T>::AddAgentStatePublishers() {
  // Adds a PoseBundle to AgentState vector translation system.
  auto pose_bundle_to_agent_state_v = builder_->template AddSystem<PoseBundleToAgentState_V>();

  // Adds an AgentState vector publisher system.
  auto aggregated_agents_state_publisher =
      builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::AgentState_V>>(kAggregatedAgentsStateTopicName);

  // Wires above's systems together and with the PoseAggregator.
  builder_->Connect(aggregator_->get_output_port(0), pose_bundle_to_agent_state_v->get_input_port(0));
  builder_->Connect(pose_bundle_to_agent_state_v->get_output_port(0),
                    aggregated_agents_state_publisher->get_input_port(0));

  const int num_agents = agents_.size();

  if (num_agents > 0) {
    // Adds an AgentState vector splitter system for the right number of agents.
    auto agents_state_splitter = builder_->template AddSystem<AgentState_v_Splitter<T>>(num_agents);

    // Wires up the vector splitter system.
    builder_->Connect(pose_bundle_to_agent_state_v->get_output_port(0), agents_state_splitter->get_input_port(0));

    auto agent_it = agents_.cbegin();
    for (int i = 0; i < num_agents; ++i) {
      // Adds an AgentState vector publisher system for each agent
      // and wires it with the splitter.
      auto agent_state_publisher = builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::AgentState>>(
          "agent/" + std::get<0>(*agent_it) + "/state");
      ++agent_it;

      builder_->Connect(agents_state_splitter->get_output_port(i), agent_state_publisher->get_input_port(0));
    }
  }
}

template <typename T>
std::unique_ptr<AgentSimulationBase<T>> AgentSimulationBaseBuilder<T>::Build() {
  // Exposes all agents' state for inspection.
  AddAgentStatePublishers();

  // Exposes simulation scene for rendering.
  SceneSystem* scene_system = AddScenePublishers();

  if (road_network_ != nullptr) {
    // Adds handler of dynamic rules.
    builder_->template AddSystem<DynamicEnvironmentHandlerSystem>(
        std::make_unique<FixedPhaseIterationHandler>(road_network_.get()));
  }

  // Builds the simulation diagram.
  std::unique_ptr<drake::systems::Diagram<T>> diagram = builder_->Build();
  diagram->set_name("simulation");

  // Instantiates and initializes the simulator itself.
  auto simulator = std::make_unique<drake::systems::Simulator<T>>(*diagram);
  simulator->set_target_realtime_rate(GetTargetRealTimeRate());
  drake::systems::Context<T>& context = simulator->get_mutable_context();
  drake::systems::RungeKutta2Integrator<T>& integrator =
      simulator->template reset_integrator<RungeKutta2Integrator<T>>(GetMaxStepSize());
  integrator.set_fixed_step_mode(UsesFixedStepMode());
  simulator->Initialize();

  // Injects simulation context references into agents.
  for (auto& name_and_agent : agents_) {
    AgentBase<T>* agent = name_and_agent.second.get();
    drake::systems::Context<T>& agent_context = diagram->GetMutableSubsystemContext(agent->GetDiagram(), &context);
    agent->SetContext(&agent_context);
  }

  // Enables caching support for the entire simulation.
  context.EnableCaching();

  // Yields simulation instance.
  if (road_geometry_ != nullptr) {
    return std::make_unique<AgentSimulationBase<T>>(std::move(simulator), std::move(diagram), std::move(agents_),
                                                    std::move(road_geometry_), scene_graph_, scene_system);
  }

  return std::make_unique<AgentSimulationBase<T>>(std::move(simulator), std::move(diagram), std::move(agents_),
                                                  std::move(road_network_), scene_graph_, scene_system);
}

template class AgentSimulationBaseBuilder<double>;

}  // namespace delphyne
