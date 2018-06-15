// Copyright 2017 Toyota Research Institute

#include <algorithm>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include <drake/automotive/gen/driving_command.h>
#include <drake/automotive/gen/driving_command_translator.h>
#include <drake/automotive/gen/maliput_railcar_state_translator.h>
#include <drake/automotive/gen/simple_car_state.h>
#include <drake/automotive/gen/simple_car_state_translator.h>
#include <drake/automotive/idm_controller.h>
#include <drake/automotive/maliput/api/junction.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/segment.h>
#include <drake/automotive/maliput/utility/generate_urdf.h>
#include <drake/automotive/prius_vis.h>
#include <drake/common/drake_throw.h>
#include <drake/common/text_logging.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/create_load_robot_message.h>
#include <drake/systems/analysis/runge_kutta2_integrator.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/primitives/multiplexer.h>

#include "backend/automotive_simulator.h"
#include "backend/ign_models_assembler.h"
#include "delphyne/macros.h"
#include "translations/drake_simple_car_state_to_ign.h"
#include "translations/ign_driving_command_to_drake.h"
#include "translations/lcm_viewer_draw_to_ign_model_v.h"
#include "translations/lcm_viewer_load_robot_to_ign_model_v.h"

namespace delphyne {

using drake::automotive::SimpleCarStateIndices;
using drake::automotive::DrivingCommandIndices;
using drake::maliput::api::Lane;
using drake::maliput::api::LaneEnd;
using drake::maliput::api::LaneId;
using drake::maliput::api::RoadGeometry;
using drake::maliput::api::RoadGeometryId;
using drake::multibody::joints::kRollPitchYaw;
using drake::systems::AbstractValue;
using drake::systems::OutputPort;
using drake::systems::rendering::PoseBundle;
using drake::systems::RungeKutta2Integrator;
using drake::systems::System;
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

  car_vis_applicator_ =
      builder_->template AddSystem<drake::automotive::CarVisApplicator<T>>();
  car_vis_applicator_->set_name("car_vis_applicator");

  scene_system_ = builder_->template AddSystem<SceneSystem>();
  scene_system_->set_name("scene_system");

  bundle_to_draw_ = builder_->template AddSystem<
      drake::systems::rendering::PoseBundleToDrawMessage>();
  bundle_to_draw_->set_name("bundle_to_draw");
}

template <typename T>
drake::systems::DiagramBuilder<T>* AutomotiveSimulator<T>::get_builder() {
  DELPHYNE_DEMAND(!has_started());
  return builder_.get();
}

template <typename T>
std::unique_ptr<ignition::msgs::Scene> AutomotiveSimulator<T>::GetScene() {
  DELPHYNE_DEMAND(simulator_ != nullptr);

  auto scene_msg = std::make_unique<ignition::msgs::Scene>();

  // The scene is the output of the scene system. We could use
  // LeafOutputPort::Eval to directly retrieve the last calculated value, but
  // that function is as of now unimplemented, so we need to recalculate it.
  // Since this doesn't happen often (once per scene request), this is not an
  // issue.
  const drake::systems::Context<T>& scene_context =
      diagram_->GetSubsystemContext(*scene_system_, simulator_->get_context());

  std::unique_ptr<SystemOutput<T>> output =
      scene_system_->AllocateOutput(scene_context);
  scene_system_->CalcOutput(scene_context, output.get());

  scene_msg->CopyFrom(
      output->get_data(0)->template GetValue<ignition::msgs::Scene>());

  return std::move(scene_msg);
}

template <typename T>
void AutomotiveSimulator<T>::ConnectCarOutputsAndPriusVis(
    int id, const OutputPort<T>& pose_output,
    const OutputPort<T>& velocity_output) {
  DELPHYNE_DEMAND(&pose_output.get_system() == &velocity_output.get_system());
  const std::string name = pose_output.get_system().get_name();
  auto ports = aggregator_->AddSinglePoseAndVelocityInput(name, id);
  builder_->Connect(pose_output, ports.pose_descriptor);
  builder_->Connect(velocity_output, ports.velocity_descriptor);
  car_vis_applicator_->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<T>>(id, name));
}

// TODO(jwnimmer-tri): Modify the various vehicle model systems to be more
// uniform so common code from the following AddFooCar() methods can be moved
// into a shared method.

template <typename T>
int AutomotiveSimulator<T>::AddAgent(
    std::unique_ptr<delphyne::AgentBase<T>> agent) {
  /*********************
   * Checks
   *********************/
  DELPHYNE_DEMAND(!has_started());
  DELPHYNE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(agent->name());

  /*********************
   * Configure Agent
   *********************/
  int id = unique_system_id_++;

  if (agent->Configure(id, road_geometry_.get(), builder_.get(), aggregator_,
                       car_vis_applicator_) < 0) {
    ignerr << "Failed to configure agent '" << agent->name() << "'"
           << std::endl;
    return -1;
  }
  agents_[id] = std::move(agent);
  return id;
}

template <typename T>
const RoadGeometry* AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const RoadGeometry> road_geometry) {
  DELPHYNE_DEMAND(!has_started());
  road_geometry_ = std::move(road_geometry);
  GenerateAndLoadRoadNetworkUrdf();
  return road_geometry_.get();
}

template <typename T>
void AutomotiveSimulator<T>::GenerateAndLoadRoadNetworkUrdf() {
  std::string filename = road_geometry_->id().string();
  std::transform(filename.begin(), filename.end(), filename.begin(),
                 [](char ch) { return ch == ' ' ? '_' : ch; });
  drake::maliput::utility::GenerateUrdfFile(
      road_geometry_.get(), "/tmp", filename,
      drake::maliput::utility::ObjFeatures());
  const std::string urdf_filepath = "/tmp/" + filename + ".urdf";
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf_filepath, drake::multibody::joints::kFixed, tree_.get());
}

template <typename T>
drake::systems::System<T>& AutomotiveSimulator<T>::GetBuilderSystemByName(
    std::string name) {
  DELPHYNE_DEMAND(!has_started());
  drake::systems::System<T>* result{nullptr};
  for (drake::systems::System<T>* system : builder_->GetMutableSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
const drake::systems::System<T>& AutomotiveSimulator<T>::GetDiagramSystemByName(
    std::string name) const {
  DELPHYNE_DEMAND(has_started());
  const drake::systems::System<T>* result{nullptr};
  for (const drake::systems::System<T>* system : diagram_->GetSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
void AutomotiveSimulator<T>::Build() {
  DELPHYNE_DEMAND(diagram_ == nullptr);

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
          "visualizer/scene_update");

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
  DELPHYNE_DEMAND(!has_started());
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
}

template <typename T>
void AutomotiveSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  simulator_->StepTo(time + time_step);
}

template <typename T>
double AutomotiveSimulator<T>::get_current_simulation_time() const {
  return drake::ExtractDoubleOrThrow(simulator_->get_context().get_time());
}

template <typename T>
void AutomotiveSimulator<T>::CheckNameUniqueness(const std::string& name) {
  for (const auto& agent : agents_) {
    if (agent.second->name() == name) {
      throw std::runtime_error("An agent named \"" + name +
                               "\" already "
                               "exists. It has id " +
                               std::to_string(agent.first) + ".");
    }
  }
}

template <typename T>
PoseBundle<T> AutomotiveSimulator<T>::GetCurrentPoses() const {
  DELPHYNE_DEMAND(has_started());
  const auto& context = simulator_->get_context();
  std::unique_ptr<SystemOutput<T>> system_output =
      diagram_->AllocateOutput(context);
  diagram_->CalcOutput(context, system_output.get());
  DELPHYNE_DEMAND(system_output->get_num_ports() == 1);
  const AbstractValue* abstract_value = system_output->get_data(0);
  const PoseBundle<T>& pose_bundle =
      abstract_value->GetValueOrThrow<PoseBundle<T>>();
  return pose_bundle;
}

template class AutomotiveSimulator<double>;

}  // namespace delphyne
