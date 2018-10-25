// Copyright 2017 Toyota Research Institute

#include "backend/automotive_simulator.h"

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <drake/automotive/maliput/utility/generate_urdf.h>
#include <drake/common/drake_throw.h>
#include <drake/common/eigen_types.h>
#include <drake/common/text_logging.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/shape_specification.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/create_load_robot_message.h>
#include <drake/systems/analysis/runge_kutta2_integrator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>

#include "backend/geometry_wiring.h"
#include "backend/ign_models_assembler.h"
#include "backend/ign_publisher_system.h"
#include "delphyne/macros.h"
#include "delphyne/protobuf/agent_state.pb.h"
#include "delphyne/protobuf/agent_state_v.pb.h"
#include "translations/agent_state_v_splitter.h"
#include "translations/ign_driving_command_to_drake.h"
#include "translations/lcm_viewer_draw_to_ign_model_v.h"
#include "translations/lcm_viewer_load_robot_to_ign_model_v.h"
#include "translations/pose_bundle_to_agent_state_v.h"
#include "visualization/prius_vis.h"
#include "visualization/simple_prius_vis.h"

namespace delphyne {

using drake::maliput::api::RoadGeometry;
using drake::systems::AbstractValue;
using drake::systems::rendering::PoseBundle;
using drake::systems::RungeKutta2Integrator;
using drake::systems::SystemOutput;

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator() {
// Avoid the many & varied 'info' level logging messages coming from drake.
//
// Note: Drake will have defined HAVE_SPDLOG if it is using that
// (see lib/cmake/spdlog/spdlog-config.cmake that was installed by drake).
//
// It would be preferable if drake did not spam downstream application
// development on info channels, but that will require a discussion and
// a large swipe at the many uses in drake.
#ifdef HAVE_SPDLOG
  drake::log()->set_level(spdlog::level::warn);
#endif

  aggregator_ =
      builder_
          ->template AddSystem<drake::systems::rendering::PoseAggregator<T>>();
  aggregator_->set_name("pose_aggregator");

  car_vis_applicator_ = builder_->template AddSystem<CarVisApplicator<T>>();
  car_vis_applicator_->set_name("car_vis_applicator");

  scene_graph_ = builder_->template AddSystem<drake::geometry::SceneGraph<T>>();
  scene_graph_->set_name("scene_graph");

  scene_system_ = builder_->template AddSystem<SceneSystem>();
  scene_system_->set_name("scene_system");

  bundle_to_draw_ = builder_->template AddSystem<
      drake::systems::rendering::PoseBundleToDrawMessage>();
  bundle_to_draw_->set_name("bundle_to_draw");
}

template <typename T>
drake::systems::DiagramBuilder<T>* AutomotiveSimulator<T>::get_builder() {
  DELPHYNE_VALIDATE(!has_started(), std::runtime_error,
                    "Cannot get the builder on a running simulation");
  return builder_.get();
}

template <typename T>
std::unique_ptr<ignition::msgs::Scene> AutomotiveSimulator<T>::GetScene() {
  DELPHYNE_VALIDATE(simulator_ != nullptr, std::runtime_error,
                    "Cannot get the scene on an uninitialized simulator");

  auto scene_msg = std::make_unique<ignition::msgs::Scene>();

  // The scene is the output of the scene system. We could use
  // LeafOutputPort::Eval to directly retrieve the last calculated value, but
  // that function is as of now unimplemented, so we need to recalculate it.
  // Since this doesn't happen often (once per scene request), this is not an
  // issue.
  const drake::systems::Context<T>& scene_context =
      diagram_->GetSubsystemContext(*scene_system_, simulator_->get_context());

  std::unique_ptr<SystemOutput<T>> output = scene_system_->AllocateOutput();
  scene_system_->CalcOutput(scene_context, output.get());

  scene_msg->CopyFrom(
      output->get_data(0)->template GetValue<ignition::msgs::Scene>());

  return std::move(scene_msg);
}

template <typename T>
delphyne::AgentBase<T>* AutomotiveSimulator<T>::AddAgent(
    std::unique_ptr<AgentBase<T>> agent) {
  /*********************
   * Checks
   *********************/
  DELPHYNE_VALIDATE(!has_started(), std::runtime_error,
                    "Cannot add an agent to a running simulation");
  CheckNameUniqueness(agent->name());

  /*********************
   * Unique ID
   *********************/
  int id = unique_system_id_++;

  /*********************
   * Agent Diagram
   *********************/
  std::unique_ptr<DiagramBundle<T>> bundle = agent->BuildDiagram();
  drake::systems::Diagram<double>* diagram =
      builder_->AddSystem(std::move(bundle->diagram));

  drake::systems::rendering::PoseVelocityInputPorts<double> ports =
      aggregator_->AddSinglePoseAndVelocityInput(agent->name(), id);
  builder_->Connect(diagram->get_output_port(bundle->outputs["pose"]),
                    ports.pose_input_port);
  builder_->Connect(diagram->get_output_port(bundle->outputs["velocity"]),
                    ports.velocity_input_port);
  builder_->Connect(aggregator_->get_output_port(0),
                    diagram->get_input_port(bundle->inputs["traffic_poses"]));

  /*********************
   * Agent Visuals
   *********************/
  // TODO(daniel.stonier) this just enforces ... 'everything is a prius'.
  // We'll need a means of having the agents report what visual they have and
  // hooking that up. Also wondering why visuals are in the drake diagram?
  car_vis_applicator_->AddCarVis(
      std::make_unique<SimplePriusVis<double>>(id, agent->name()));
  /*********************
   * Agent Geometry
   *********************/
  // Wires up the Prius geometry.
  builder_->Connect(
      diagram->get_output_port(bundle->outputs["pose"]),
      WirePriusGeometry(agent->name(), agent->initial_world_pose(),
                        builder_.get(), scene_graph_,
                        &(agent->mutable_geometry_ids())));

  // save a handle to it
  agents_[id] = std::move(agent);

  return agents_[id].get();
}

template <typename T>
const RoadGeometry* AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const RoadGeometry> road_geometry) {
  drake::maliput::utility::ObjFeatures features;
  // Max distance between rendered vertices (in s- or r-dimension), in meters.
  features.max_grid_unit = 1.0;
  // Min number of vertices (in s- or r-dimension).
  features.min_grid_resolution = 5.0;
  features.draw_elevation_bounds = false;
  features.draw_stripes = true;
  features.draw_arrows = false;
  features.draw_lane_haze = false;
  features.draw_branch_points = false;
  return SetRoadGeometry(std::move(road_geometry), features);
}

template <typename T>
const RoadGeometry* AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const RoadGeometry> road_geometry,
    const drake::maliput::utility::ObjFeatures& features) {
  DELPHYNE_VALIDATE(!has_started(), std::runtime_error,
                    "Cannot set a road geometry on a running simulation");
  road_geometry_ = std::move(road_geometry);
  GenerateAndLoadRoadNetworkUrdf(features);
  return road_geometry_.get();
}

template <typename T>
void AutomotiveSimulator<T>::GenerateAndLoadRoadNetworkUrdf(
    const drake::maliput::utility::ObjFeatures& features) {
  std::string filename = road_geometry_->id().string();
  std::transform(filename.begin(), filename.end(), filename.begin(),
                 [](char ch) { return ch == ' ' ? '_' : ch; });
  drake::maliput::utility::GenerateUrdfFile(road_geometry_.get(), "/tmp",
                                            filename, features);
  const std::string urdf_filepath = "/tmp/" + filename + ".urdf";
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf_filepath, drake::multibody::joints::kFixed, tree_.get());
}

namespace {

// A helper functor to check for a given object source (e.g. a GeometryId)
// on agent ID to agent mappings. Useful in combination with the STL algorithm
// library.
//
// @tparam T Agents' scalar type.
// @tparam U Object type.
// @see delphyne::AgentBase::is_source_of()
template <typename T, typename U>
struct IsSourceOf {
  // Constructs the functor with given @p object_in reference.
  // @remarks As only a reference to the object is kept (to keep it
  //          lightweight), the caller MUST ensure that the object
  //          instance outlives the functor instance.
  explicit IsSourceOf(const U& object_in) : object(object_in) {}

  // Checks whether the given (agent ID, agent) pair is the source of the
  // associated `object`.
  bool operator()(
      const std::pair<const int, std::unique_ptr<AgentBase<T>>>& id_agent) {
    return id_agent.second->is_source_of(object);
  }

  // Associated object to check the source of.
  const U& object;
};

}  // namespace

template <typename T>
const std::vector<std::pair<delphyne::AgentBase<T>*, delphyne::AgentBase<T>*>>
AutomotiveSimulator<T>::GetCollisions() {
  DELPHYNE_VALIDATE(has_started(), std::runtime_error,
                    "Can only get collisions on a running simulation");
  using drake::geometry::GeometryId;
  using drake::geometry::QueryObject;
  using drake::geometry::PenetrationAsPointPair;
  const std::vector<PenetrationAsPointPair<T>> collisions =
      scene_query_->GetValue<QueryObject<T>>().ComputePointPairPenetration();
  std::vector<std::pair<delphyne::AgentBase<T>*, delphyne::AgentBase<T>*>>
      agents_colliding;
  for (const auto& collision : collisions) {
    const auto it_A = std::find_if(agents_.begin(), agents_.end(),
                                   IsSourceOf<T, GeometryId>(collision.id_A));
    DELPHYNE_VALIDATE(it_A != agents_.end(), std::runtime_error,
                      "Could not find first agent in list of agents");
    const auto it_B = std::find_if(agents_.begin(), agents_.end(),
                                   IsSourceOf<T, GeometryId>(collision.id_B));
    DELPHYNE_VALIDATE(it_B != agents_.end(), std::runtime_error,
                      "Could not find second agent in list of agents");
    agents_colliding.emplace_back(agents_[it_A->first].get(),
                                  agents_[it_B->first].get());
  }
  return agents_colliding;
}

template <typename T>
void AutomotiveSimulator<T>::Build() {
  DELPHYNE_VALIDATE(diagram_ == nullptr, std::runtime_error,
                    "Cannot call Build more than once");

  builder_->Connect(aggregator_->get_output_port(0),
                    car_vis_applicator_->get_car_poses_input_port());

  // The bundle of poses are translated into an LCM viewer draw message.
  builder_->Connect(
      car_vis_applicator_->get_visual_geometry_poses_output_port(),
      bundle_to_draw_->get_input_port(0));

  // The LCM viewer draw message is translated into an ignition Model_V message.
  auto viewer_draw_translator =
      builder_->template AddSystem<LcmViewerDrawToIgnModelV>();
  builder_->Connect(*bundle_to_draw_, *viewer_draw_translator);

  // The translated Model_V message is then published.
  auto model_v_publisher =
      builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::Model_V>>(
          "visualizer/scene_update", kSceneUpdatesPublishRateHz);

  // The translated ignition message is then published.
  builder_->Connect(*viewer_draw_translator, *model_v_publisher);

  // An ignition scene message is built from the geometry description, and both
  // model and links' updated poses.

  // The geometry description is retrieved from multiple sources as LCM viewer
  // load robot messages, so those need to be aggregated. These messages are not
  // obtained from system output ports, but from method calls, so lambdas that
  // call them are stored in the system input port after the diagram has been
  // built.
  load_robot_aggregator_ = builder_->template AddSystem<LoadRobotAggregator>();

  // The aggregated LCM viewer load robot message containing the geometry
  // description is translated into an ignition Model_V message.
  auto viewer_load_robot_translator =
      builder_->template AddSystem<LcmViewerLoadRobotToIgnModelV>();
  builder_->Connect(*load_robot_aggregator_, *viewer_load_robot_translator);

  // The Model_V describing the geometry is finally used to build the scene.
  builder_->Connect(viewer_load_robot_translator->get_output_port(0),
                    scene_system_->get_geometry_models_input_port());

  // Updated model and links poses are stored in the Model_V message that
  // is assembled from an LCM viewer draw translation and a pose bundle.
  auto models_assembler = builder_->template AddSystem<IgnModelsAssembler>();

  builder_->Connect(viewer_draw_translator->get_output_port(0),
                    models_assembler->get_models_input_port());

  builder_->Connect(aggregator_->get_output_port(0),
                    models_assembler->get_states_input_port());

  builder_->Connect(models_assembler->get_output_port(0),
                    scene_system_->get_updated_pose_models_input_port());

  // The scene is then published over a scene topic to update the scene tree
  // widget of the visualizer. Because this information is not needed at the
  // same frequency the simulation runs at, the publishing frequency is reduced.
  auto scene_publisher =
      builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::Scene>>(
          "scene", kSceneTreePublishRateHz);
  builder_->Connect(*scene_system_, *scene_publisher);

  pose_bundle_output_port_ =
      builder_->ExportOutput(aggregator_->get_output_port(0));

  // Defines a AgentState_V channel name.
  const std::string vectorized_agent_state_channel = "agents/state";

  typedef IgnPublisherSystem<ignition::msgs::AgentState_V>
      VectorizedAgentsStatePublisherSystem;

  // Creates a PoseBundleToAgentState_V system.
  PoseBundleToAgentState_V* pose_bundle_to_agent_state_v_ =
      builder_->template AddSystem<PoseBundleToAgentState_V>(
          std::make_unique<PoseBundleToAgentState_V>());

  VectorizedAgentsStatePublisherSystem*
      vectorized_agents_state_publisher_system =
          builder_->template AddSystem<VectorizedAgentsStatePublisherSystem>(
              std::make_unique<VectorizedAgentsStatePublisherSystem>(
                  vectorized_agent_state_channel));

  // Connects the PoseBundleToAgentState_v input and output.
  builder_->Connect(aggregator_->get_output_port(0),
                    pose_bundle_to_agent_state_v_->get_input_port(0));
  builder_->Connect(
      pose_bundle_to_agent_state_v_->get_output_port(0),
      vectorized_agents_state_publisher_system->get_input_port(0));

  // Makes the AgentState publisher type be more readable.
  typedef IgnPublisherSystem<ignition::msgs::AgentState>
      AgentsStatePublisherSystem;

  // Retrieves the number of agents within the system.
  const int num_agents = agents_.size();

  if (num_agents > 0) {
    // Creates the splitter with the right number of agents.
    AgentState_v_Splitter<double>* splitter =
        builder_->template AddSystem<AgentState_v_Splitter<double>>(
            std::make_unique<AgentState_v_Splitter<double>>(num_agents));

    builder_->Connect(pose_bundle_to_agent_state_v_->get_output_port(0),
                      splitter->get_input_port(0));

    for (int i = 0; i < num_agents; ++i) {
      // Adds a new AgentState Splitter system for each agent found
      // and connects it to its corresponding output of the splitter.
      builder_->Connect(splitter->get_output_port(i),
                        builder_
                            ->template AddSystem<AgentsStatePublisherSystem>(
                                std::make_unique<AgentsStatePublisherSystem>(
                                    "agent/" + std::to_string(i) + "/state"))
                            ->get_input_port(0));
    }
  }

  diagram_ = builder_->Build();
  diagram_->set_name("AutomotiveSimulator");
}

template <typename T>
void AutomotiveSimulator<T>::InitializeSceneGeometryAggregator() {
  std::vector<std::function<drake::lcmt_viewer_load_robot()>>
      load_robot_generators{
          [this]() { return car_vis_applicator_->get_load_robot_message(); },
          [this]() {
            return drake::multibody::CreateLoadRobotMessage<T>(*tree_);
          }};

  drake::systems::Context<T>& aggregator_context =
      diagram_->GetMutableSubsystemContext(*load_robot_aggregator_,
                                           &simulator_->get_mutable_context());

  const double kOutputPortIndex{0};
  aggregator_context.FixInputPort(
      kOutputPortIndex,
      drake::systems::AbstractValue::Make(load_robot_generators));
}

template <typename T>
void AutomotiveSimulator<T>::Start(double realtime_rate) {
  DELPHYNE_VALIDATE(!has_started(), std::runtime_error,
                    "Cannot start an already running simulation");
  if (diagram_ == nullptr) {
    Build();
  }

  simulator_ = std::make_unique<drake::systems::Simulator<T>>(*diagram_);

  InitializeSceneGeometryAggregator();

  simulator_->set_target_realtime_rate(realtime_rate);
  const double max_step_size = 0.01;
  simulator_->template reset_integrator<RungeKutta2Integrator<T>>(
      *diagram_, max_step_size, &simulator_->get_mutable_context());
  simulator_->get_mutable_integrator()->set_fixed_step_mode(true);
  simulator_->Initialize();

  // Enable caching support.
  simulator_->get_mutable_context().EnableCaching();

  // Retrieve SceneGraph query object for later use.
  const drake::systems::OutputPort<T>& scene_query_port =
      scene_graph_->get_query_output_port();
  scene_query_ = scene_query_port.Allocate();
  scene_query_port.Calc(
      diagram_->GetSubsystemContext(*scene_graph_, simulator_->get_context()),
      scene_query_.get());
}

template <typename T>
void AutomotiveSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  simulator_->StepTo(time + time_step);
}

template <typename T>
double AutomotiveSimulator<T>::GetCurrentSimulationTime() const {
  return drake::ExtractDoubleOrThrow(simulator_->get_context().get_time());
}

template <typename T>
void AutomotiveSimulator<T>::CheckNameUniqueness(const std::string& name) {
  for (const auto& agent : agents_) {
    DELPHYNE_VALIDATE(agent.second->name() != name, std::runtime_error,
                      "An agent named \"" + name +
                          "\" already exists. It has id " +
                          std::to_string(agent.first) + ".");
  }
}

template <typename T>
PoseBundle<T> AutomotiveSimulator<T>::GetCurrentPoses() const {
  DELPHYNE_VALIDATE(
      has_started(), std::runtime_error,
      "Cannot get poses for a simulation that has not yet started");
  const auto& context = simulator_->get_context();
  std::unique_ptr<SystemOutput<T>> system_output = diagram_->AllocateOutput();
  diagram_->CalcOutput(context, system_output.get());
  DELPHYNE_VALIDATE(system_output->get_num_ports() == 1, std::runtime_error,
                    "System output has too many ports");
  const AbstractValue* abstract_value = system_output->get_data(0);
  const PoseBundle<T>& pose_bundle =
      abstract_value->GetValueOrThrow<PoseBundle<T>>();
  return pose_bundle;
}

template <typename T>
drake::systems::Context<T>* AutomotiveSimulator<T>::GetMutableContext() {
  return &simulator_->get_mutable_context();
}

template class AutomotiveSimulator<double>;

}  // namespace delphyne
