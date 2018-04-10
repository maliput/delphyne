// Copyright 2017 Toyota Research Institute

#include <algorithm>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/maliput_railcar_state_translator.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/primitives/multiplexer.h"

#include "backend/agent_plugin_loader.h"
#include "backend/automotive_simulator.h"
#include "backend/linb-any"
#include "backend/system.h"
#include "backend/translation_systems/drake_simple_car_state_to_ign.h"
#include "backend/translation_systems/ign_driving_command_to_drake.h"
#include "backend/translation_systems/lcm_viewer_draw_to_ign_model_v.h"
#include "backend/translation_systems/lcm_viewer_load_robot_to_ign_model_v.h"

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
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::OutputPort;
using drake::systems::rendering::PoseBundle;
using drake::systems::RungeKutta2Integrator;
using drake::systems::System;
using drake::systems::SystemOutput;

namespace backend {

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator()
    : AutomotiveSimulator(std::make_unique<drake::lcm::DrakeLcm>()) {}

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator(
    std::unique_ptr<drake::lcm::DrakeLcmInterface> lcm)
    : lcm_(std::move(lcm)) {
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
AutomotiveSimulator<T>::~AutomotiveSimulator() {
  // Forces the LCM instance to be destroyed before any of the subscribers are
  // destroyed.
  lcm_.reset();
}

template <typename T>
drake::lcm::DrakeLcmInterface* AutomotiveSimulator<T>::get_lcm() {
  return lcm_.get();
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
int AutomotiveSimulator<T>::AddLoadableCar(
    const std::string& plugin,
    const std::map<std::string, linb::any>& parameters, const std::string& name,
    drake::systems::BasicVector<T>* initial_state) {
  DELPHYNE_DEMAND(!has_started());
  DELPHYNE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  int id = allocate_vehicle_number();

  std::unique_ptr<delphyne::backend::AgentPluginBase<T>> agent =
      delphyne::backend::LoadPlugin<T>(plugin);
  if (agent == nullptr) {
    return -1;
  }

  auto car =
      builder_->template AddSystem<delphyne::backend::AgentPluginBase<T>>(
          std::move(agent));
  car->set_name(name);
  vehicles_[id] = car;

  loadable_car_initial_states_[car] = initial_state;
  if (car->Configure(parameters, builder_.get(), lcm_.get(), name, id,
                     aggregator_, car_vis_applicator_) < 0) {
    return -1;
  }

  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusSimpleCar(
    const std::string& name, const std::string& channel_name,
    const drake::automotive::SimpleCarState<T>& initial_state) {
  DELPHYNE_DEMAND(!has_started());
  DELPHYNE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  const int id = allocate_vehicle_number();

  DELPHYNE_DEMAND(!channel_name.empty());

  // Subscribes to ignition driving command messages.
  auto driving_command_subscriber = builder_->template AddSystem<
      IgnSubscriberSystem<ignition::msgs::AutomotiveDrivingCommand>>(
      channel_name);

  auto driving_command_translator =
      builder_
          ->template AddSystem<translation_systems::IgnDrivingCommandToDrake>();

  // Those messages are then translated to Drake driving command messages.
  builder_->Connect(*driving_command_subscriber, *driving_command_translator);

  auto simple_car =
      builder_->template AddSystem<drake::automotive::SimpleCar<T>>();
  simple_car->set_name(name);

  // The translated Drake driving command messages are then sent to the car.
  builder_->Connect(*driving_command_translator, *simple_car);

  vehicles_[id] = simple_car;
  simple_car_initial_states_[simple_car].set_value(initial_state.get_value());

  ConnectCarOutputsAndPriusVis(id, simple_car->pose_output(),
                               simple_car->velocity_output());

  AddPublisher(*simple_car, id);
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddMobilControlledSimpleCar(
    const std::string& name, bool initial_with_s,
    const drake::automotive::SimpleCarState<T>& initial_state) {
  DELPHYNE_DEMAND(!has_started());
  DELPHYNE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  if (road_ == nullptr) {
    throw std::runtime_error(
        "AutomotiveSimulator::AddMobilControlledSimpleCar(): "
        "RoadGeometry not set. Please call SetRoadGeometry() first before "
        "calling this method.");
  }
  const int id = allocate_vehicle_number();

  auto mobil_planner =
      builder_->template AddSystem<drake::automotive::MobilPlanner<T>>(
          *road_, initial_with_s,
          drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
          0. /* time period (unused) */);
  mobil_planner->set_name(name + "_mobil_planner");
  auto idm_controller =
      builder_->template AddSystem<drake::automotive::IdmController<T>>(
          *road_, drake::automotive::ScanStrategy::kBranches,
          drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
          0. /* time period (unused) */);
  idm_controller->set_name(name + "_idm_controller");

  auto simple_car =
      builder_->template AddSystem<drake::automotive::SimpleCar<T>>();
  simple_car->set_name(name + "_simple_car");
  vehicles_[id] = simple_car;
  simple_car_initial_states_[simple_car].set_value(initial_state.get_value());
  auto pursuit =
      builder_
          ->template AddSystem<drake::automotive::PurePursuitController<T>>();
  pursuit->set_name(name + "_pure_pursuit_controller");
  auto mux = builder_->template AddSystem<drake::systems::Multiplexer<T>>(
      drake::automotive::DrivingCommand<T>());
  mux->set_name(name + "_mux");

  // Wire up MobilPlanner and IdmController.
  builder_->Connect(simple_car->pose_output(), mobil_planner->ego_pose_input());
  builder_->Connect(simple_car->velocity_output(),
                    mobil_planner->ego_velocity_input());
  builder_->Connect(idm_controller->acceleration_output(),
                    mobil_planner->ego_acceleration_input());
  builder_->Connect(aggregator_->get_output_port(0),
                    mobil_planner->traffic_input());

  builder_->Connect(simple_car->pose_output(),
                    idm_controller->ego_pose_input());
  builder_->Connect(simple_car->velocity_output(),
                    idm_controller->ego_velocity_input());
  builder_->Connect(aggregator_->get_output_port(0),
                    idm_controller->traffic_input());

  builder_->Connect(simple_car->pose_output(), pursuit->ego_pose_input());
  builder_->Connect(mobil_planner->lane_output(), pursuit->lane_input());
  // Build DrivingCommand via a mux of two scalar outputs (a BasicVector where
  // row 0 = steering command, row 1 = acceleration command).
  builder_->Connect(pursuit->steering_command_output(), mux->get_input_port(0));
  builder_->Connect(idm_controller->acceleration_output(),
                    mux->get_input_port(1));
  builder_->Connect(mux->get_output_port(0), simple_car->get_input_port(0));

  ConnectCarOutputsAndPriusVis(id, simple_car->pose_output(),
                               simple_car->velocity_output());

  AddPublisher(*simple_car, id);
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusTrajectoryCar(
    const std::string& name, const drake::automotive::Curve2<double>& curve,
    double speed, double start_position) {
  DELPHYNE_DEMAND(!has_started());
  DELPHYNE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  const int id = allocate_vehicle_number();

  auto trajectory_car =
      builder_->template AddSystem<drake::automotive::TrajectoryCar<T>>(curve);
  trajectory_car->set_name(name);
  vehicles_[id] = trajectory_car;

  drake::automotive::TrajectoryCarState<double> initial_state;
  initial_state.set_position(start_position);
  initial_state.set_speed(speed);
  trajectory_car_initial_states_[trajectory_car].set_value(
      initial_state.get_value());

  ConnectCarOutputsAndPriusVis(id, trajectory_car->pose_output(),
                               trajectory_car->velocity_output());

  AddPublisher(*trajectory_car, id);
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusMaliputRailcar(
    const std::string& name,
    const drake::automotive::LaneDirection& initial_lane_direction,
    const drake::automotive::MaliputRailcarParams<T>& params,
    const drake::automotive::MaliputRailcarState<T>& initial_state) {
  DELPHYNE_DEMAND(!has_started());
  DELPHYNE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  if (road_ == nullptr) {
    throw std::runtime_error(
        "AutomotiveSimulator::AddPriusMaliputRailcar(): "
        "RoadGeometry not set. Please call SetRoadGeometry() first before "
        "calling this method.");
  }
  if (initial_lane_direction.lane == nullptr) {
    throw std::runtime_error(
        "AutomotiveSimulator::AddPriusMaliputRailcar(): "
        "The provided initial lane is nullptr.");
  }
  if (initial_lane_direction.lane->segment()->junction()->road_geometry() !=
      road_.get()) {
    throw std::runtime_error(
        "AutomotiveSimulator::AddPriusMaliputRailcar(): "
        "The provided initial lane is not within this simulation's "
        "RoadGeometry.");
  }

  const int id = allocate_vehicle_number();

  auto railcar =
      builder_->template AddSystem<drake::automotive::MaliputRailcar<T>>(
          initial_lane_direction);
  railcar->set_name(name);
  vehicles_[id] = railcar;
  railcar_configs_[railcar].first.set_value(params.get_value());
  railcar_configs_[railcar].second.set_value(initial_state.get_value());

  ConnectCarOutputsAndPriusVis(id, railcar->pose_output(),
                               railcar->velocity_output());
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddIdmControlledPriusMaliputRailcar(
    const std::string& name,
    const drake::automotive::LaneDirection& initial_lane_direction,
    const drake::automotive::MaliputRailcarParams<T>& params,
    const drake::automotive::MaliputRailcarState<T>& initial_state) {
  const int id = AddPriusMaliputRailcar(name, initial_lane_direction, params,
                                        initial_state);
  const drake::automotive::MaliputRailcar<T>* railcar =
      dynamic_cast<const drake::automotive::MaliputRailcar<T>*>(
          vehicles_.at(id));
  DELPHYNE_DEMAND(railcar != nullptr);

  auto controller =
      builder_->template AddSystem<drake::automotive::IdmController<T>>(
          *road_, drake::automotive::ScanStrategy::kBranches,
          drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
          0. /* time period (unused) */);
  controller->set_name(name + "_IdmController");

  builder_->Connect(railcar->pose_output(), controller->ego_pose_input());
  builder_->Connect(railcar->velocity_output(),
                    controller->ego_velocity_input());
  builder_->Connect(aggregator_->get_output_port(0),
                    controller->traffic_input());
  builder_->Connect(controller->acceleration_output(),
                    railcar->command_input());
  return id;
}

template <typename T>
void AutomotiveSimulator<T>::SetMaliputRailcarAccelerationCommand(
    int id, double acceleration) {
  DELPHYNE_DEMAND(has_started());
  const auto iterator = vehicles_.find(id);
  if (iterator == vehicles_.end()) {
    throw std::runtime_error(
        "AutomotiveSimulator::"
        "SetMaliputRailcarAccelerationCommand(): Failed to find vehicle with "
        "id " +
        std::to_string(id) + ".");
  }
  drake::automotive::MaliputRailcar<T>* railcar =
      dynamic_cast<drake::automotive::MaliputRailcar<T>*>(iterator->second);
  if (railcar == nullptr) {
    throw std::runtime_error(
        "AutomotiveSimulator::"
        "SetMaliputRailcarAccelerationCommand(): The vehicle with "
        "id " +
        std::to_string(id) + " was not a MaliputRailcar.");
  }
  DELPHYNE_ASSERT(diagram_ != nullptr);
  DELPHYNE_ASSERT(simulator_ != nullptr);
  drake::systems::Context<T>& context = diagram_->GetMutableSubsystemContext(
      *railcar, &simulator_->get_mutable_context());
  context.FixInputPort(railcar->command_input().get_index(),
                       drake::systems::BasicVector<double>::Make(acceleration));
}

template <typename T>
const RoadGeometry* AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const RoadGeometry> road) {
  DELPHYNE_DEMAND(!has_started());
  road_ = std::move(road);
  GenerateAndLoadRoadNetworkUrdf();
  return road_.get();
}

template <typename T>
const drake::maliput::api::Lane* AutomotiveSimulator<T>::FindLane(
    const std::string& name) const {
  if (road_ == nullptr) {
    throw std::runtime_error(
        "AutomotiveSimulator::FindLane(): RoadGeometry "
        "not set. Please call SetRoadGeometry() first before calling this "
        "method.");
  }
  for (int i = 0; i < road_->num_junctions(); ++i) {
    const drake::maliput::api::Junction* junction = road_->junction(i);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const drake::maliput::api::Segment* segment = junction->segment(j);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const drake::maliput::api::Lane* lane = segment->lane(k);
        if (lane->id() == LaneId(name)) {
          return lane;
        }
      }
    }
  }
  throw std::runtime_error(
      "AutomotiveSimulator::FindLane(): Failed to find "
      "lane named \"" +
      name + "\".");
}

template <typename T>
void AutomotiveSimulator<T>::GenerateAndLoadRoadNetworkUrdf() {
  std::string filename = road_->id().string();
  std::transform(filename.begin(), filename.end(), filename.begin(),
                 [](char ch) { return ch == ' ' ? '_' : ch; });
  drake::maliput::utility::GenerateUrdfFile(
      road_.get(), "/tmp", filename, drake::maliput::utility::ObjFeatures());
  const std::string urdf_filepath = "/tmp/" + filename + ".urdf";
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf_filepath, drake::multibody::joints::kFixed, tree_.get());
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const drake::automotive::MaliputRailcar<T>& system, int vehicle_number) {
  DELPHYNE_DEMAND(!has_started());

  // TODO(nventuro): add a translator to ignition. See #302
  static const drake::automotive::MaliputRailcarStateTranslator translator;
  const std::string channel =
      std::to_string(vehicle_number) + "_MALIPUT_RAILCAR_STATE";
  auto publisher = builder_->template AddSystem<LcmPublisherSystem>(
      channel, translator, lcm_.get());
  builder_->Connect(system.state_output(), publisher->get_input_port());
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const drake::automotive::SimpleCar<T>& system, int vehicle_number) {
  DELPHYNE_DEMAND(!has_started());
  auto simple_car_translator =
      builder_
          ->template AddSystem<translation_systems::DrakeSimpleCarStateToIgn>();

  // The car state is first translated into an ignition car state.
  builder_->Connect(system.state_output(),
                    simple_car_translator->get_input_port(0));

  const std::string channel =
      "agents/" + std::to_string(vehicle_number) + "/state";
  auto simple_car_publisher = builder_->template AddSystem<
      IgnPublisherSystem<ignition::msgs::SimpleCarState>>(channel);

  // And the translated ignition car state is then published.
  builder_->Connect(*simple_car_translator, *simple_car_publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const drake::automotive::TrajectoryCar<T>& system, int vehicle_number) {
  DELPHYNE_DEMAND(!has_started());

  // TODO(nventuro): add a translator to ignition. See #302
  static const drake::automotive::SimpleCarStateTranslator translator;
  const std::string channel =
      std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE";
  auto publisher = builder_->template AddSystem<LcmPublisherSystem>(
      channel, translator, lcm_.get());
  builder_->Connect(system.raw_pose_output(), publisher->get_input_port());
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
      builder_
          ->template AddSystem<translation_systems::LcmViewerDrawToIgnModelV>();
  builder_->Connect(*bundle_to_draw_, *viewer_draw_translator);

  // The translated Model_V message is then published.
  auto model_v_publisher =
      builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::Model_V>>(
          "visualizer/scene_update");

  // The translated ignition message is then published.
  builder_->Connect(*viewer_draw_translator, *model_v_publisher);

  // An ignition scene message is built from the geometry description, and the
  // updated poses.

  // The geometry description is retrieved from multiple sources as LCM viewer
  // load robot messages, so those need to be aggregated. These messages are not
  // obtained from system output ports, but from method calls, so lambdas that
  // call them are stored in the system input port after the diagram has been
  // built.
  load_robot_aggregator_ = builder_->template AddSystem<LoadRobotAggregator>();

  // The aggregated LCM viewer load robot message containing the geometry
  // description is translated into an ignition Model_V message.
  auto viewer_load_robot_translator = builder_->template AddSystem<
      translation_systems::LcmViewerLoadRobotToIgnModelV>();
  builder_->Connect(*load_robot_aggregator_, *viewer_load_robot_translator);

  // The Model_V describing the geometry is finally used to build the scene.
  builder_->Connect(viewer_load_robot_translator->get_output_port(0),
                    scene_system_->get_geometry_models_input_port());

  // The updated poses are stored in the Model_V message obtained from
  // translating an LCM viewer draw.
  builder_->Connect(viewer_draw_translator->get_output_port(0),
                    scene_system_->get_updated_pose_models_input_port());

  // The scene is then published over a scene topic to update the scene tree
  // widget of the visualizer. Because this information is not needed at the
  // same frequency the simulation runs at, the publishing frequency is reduced.

  // TODO(basicNew): Temporary disabling this as it is breaking the UI. To be
  // fixed ASAP. Issue recorded in
  // https://github.com/ToyotaResearchInstitute/delphyne/issues/324
  //
  // auto scene_publisher =
  //     builder_->template AddSystem<IgnPublisherSystem<ignition::msgs::Scene>>(
  //         "scene", kScenePublishPeriodMs);
  // builder_->Connect(*scene_system_, *scene_publisher);

  pose_bundle_output_port_ =
      builder_->ExportOutput(aggregator_->get_output_port(0));

  diagram_ = builder_->Build();
  diagram_->set_name("AutomotiveSimulator");
}

template <typename T>
void AutomotiveSimulator<T>::InitializeTrajectoryCars() {
  for (const auto& pair : trajectory_car_initial_states_) {
    const drake::automotive::TrajectoryCar<T>* const car = pair.first;
    const drake::automotive::TrajectoryCarState<T>& initial_state = pair.second;

    drake::systems::VectorBase<T>& context_state =
        diagram_
            ->GetMutableSubsystemContext(*car,
                                         &simulator_->get_mutable_context())
            .get_mutable_continuous_state()
            .get_mutable_vector();
    drake::automotive::TrajectoryCarState<T>* const state =
        dynamic_cast<drake::automotive::TrajectoryCarState<T>*>(&context_state);
    DELPHYNE_ASSERT(state);
    state->set_value(initial_state.get_value());
  }
}

template <typename T>
void AutomotiveSimulator<T>::InitializeSimpleCars() {
  for (const auto& pair : simple_car_initial_states_) {
    const drake::systems::System<T>* const car = pair.first;
    const drake::automotive::SimpleCarState<T>& initial_state = pair.second;

    drake::systems::VectorBase<T>& context_state =
        diagram_
            ->GetMutableSubsystemContext(*car,
                                         &simulator_->get_mutable_context())
            .get_mutable_continuous_state()
            .get_mutable_vector();
    drake::automotive::SimpleCarState<T>* const state =
        dynamic_cast<drake::automotive::SimpleCarState<T>*>(&context_state);
    DELPHYNE_ASSERT(state);
    state->set_value(initial_state.get_value());
  }
}

template <typename T>
void AutomotiveSimulator<T>::InitializeMaliputRailcars() {
  for (auto& pair : railcar_configs_) {
    const drake::automotive::MaliputRailcar<T>* const car = pair.first;
    const drake::automotive::MaliputRailcarParams<T>& params =
        pair.second.first;
    const drake::automotive::MaliputRailcarState<T>& initial_state =
        pair.second.second;

    drake::systems::Context<T>& context = diagram_->GetMutableSubsystemContext(
        *car, &simulator_->get_mutable_context());

    drake::systems::VectorBase<T>& context_state =
        context.get_mutable_continuous_state().get_mutable_vector();
    drake::automotive::MaliputRailcarState<T>* const state =
        dynamic_cast<drake::automotive::MaliputRailcarState<T>*>(
            &context_state);
    DELPHYNE_ASSERT(state);
    state->set_value(initial_state.get_value());

    drake::automotive::MaliputRailcarParams<T>& railcar_system_params =
        car->get_mutable_parameters(&context);
    railcar_system_params.set_value(params.get_value());
  }
}

template <typename T>
void AutomotiveSimulator<T>::InitializeLoadableCars() {
  for (const auto& pair : loadable_car_initial_states_) {
    delphyne::backend::AgentPluginBase<T>* const car =
        dynamic_cast<delphyne::backend::AgentPluginBase<T>*>(pair.first);
    const drake::systems::BasicVector<T>* initial_state = pair.second;

    drake::systems::Context<T>& context = diagram_->GetMutableSubsystemContext(
        *car, &simulator_->get_mutable_context());

    drake::systems::VectorBase<T>& context_state =
        context.get_mutable_continuous_state().get_mutable_vector();
    drake::systems::BasicVector<T>* const state =
        dynamic_cast<drake::systems::BasicVector<T>*>(&context_state);
    DELPHYNE_ASSERT(state);
    state->set_value(initial_state->get_value());

    car->Initialize(&context);
  }
}

template <typename T>
void AutomotiveSimulator<T>::Start(double realtime_rate) {
  DELPHYNE_DEMAND(!has_started());
  if (diagram_ == nullptr) {
    Build();
  }

  simulator_ = std::make_unique<drake::systems::Simulator<T>>(*diagram_);

  // With a built diagram and simulator, the context of the geometry aggregator
  // system can be retrieved, and the input port fixed.
  std::vector<std::function<drake::lcmt_viewer_load_robot()>>
      load_robot_generators{
          [this]() { return car_vis_applicator_->get_load_robot_message(); },
          [this]() {
            return drake::multibody::CreateLoadRobotMessage<T>(*tree_);
          }};

  drake::systems::Context<T>& aggregator_context =
      diagram_->GetMutableSubsystemContext(*load_robot_aggregator_,
                                           &simulator_->get_mutable_context());
  aggregator_context.FixInputPort(
      0, drake::systems::AbstractValue::Make(load_robot_generators));

  InitializeTrajectoryCars();
  InitializeSimpleCars();
  InitializeMaliputRailcars();
  InitializeLoadableCars();

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
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template <typename T>
double AutomotiveSimulator<T>::get_current_simulation_time() const {
  return drake::ExtractDoubleOrThrow(simulator_->get_context().get_time());
}

template <typename T>
int AutomotiveSimulator<T>::allocate_vehicle_number() {
  DELPHYNE_DEMAND(!has_started());
  return next_vehicle_number_++;
}

template <typename T>
void AutomotiveSimulator<T>::CheckNameUniqueness(const std::string& name) {
  for (const auto& vehicle : vehicles_) {
    if (vehicle.second->get_name() == name) {
      throw std::runtime_error("A vehicle named \"" + name +
                               "\" already "
                               "exists. It has id " +
                               std::to_string(vehicle.first) + ".");
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

}  // namespace backend
}  // namespace delphyne
