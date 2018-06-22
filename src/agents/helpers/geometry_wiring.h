// Copyright 2018 Toyota Research Institute
#pragma once

#include <memory>
#include <set>
#include <string>
#include <utility>

#include <drake/common/eigen_types.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/shape_specification.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include "agents/helpers/frame_pose_aggregator.h"

namespace delphyne {

namespace detail {

/// Combines drake::geometry::FrameId name from @p root (i.e. prefix)
/// and @p leaf names.
inline std::string ResolveFrameName(const std::string& root,
                                    const std::string& leaf) {
  if (root.empty()) return leaf;
  return root + "_" + leaf;
}

}  // namespace detail

/// Wires up a Prius car geometry (and associated systems) using the given
/// @p builder and @p scene_graph.
///
/// @param frame_root A root name (i.e. a prefix) for all Prius car frames.
/// @param initial_world_to_car_transform The initial world to Prius car
///                                       transform.
/// @param builder The builder used to wire associated systems into
///                the diagram.
/// @param scene_graph The scene graph where geometries are registered into.
/// @param geometry_ids Output set of registered geometry IDs.
/// @return The Prius car drake::systems::rendering::PoseVector (abstract) port
///         to update its pose (i.e. the pose of the whole care frame) in the
///         world.
/// @tparam T A valid Eigen scalar type.
// TODO(hidmic): Extend support for non-double types when SceneGraph supports
// them.
template <typename T,
          typename std::enable_if<
            std::is_same<T, double>::value, int>::type = 0>
const drake::systems::InputPortDescriptor<T>& WirePriusGeometry(
    const std::string& frame_root,
    const drake::Isometry3<T>& initial_world_to_car_transform,
    drake::systems::DiagramBuilder<T>* builder,
    drake::geometry::SceneGraph<T>* scene_graph,
    std::set<drake::geometry::GeometryId>* geometry_ids) {
  using drake::Isometry3;
  using drake::Quaternion;
  using drake::Translation3;
  using drake::geometry::Box;
  using drake::geometry::SourceId;
  using drake::geometry::FrameId;
  using drake::geometry::GeometryFrame;
  using drake::geometry::GeometryInstance;
  using drake::systems::ConstantVectorSource;
  using drake::systems::rendering::PoseVector;

  // Prius' dimensional constants have been retrieved or computed from its SDF
  // (currently living in Drake).
  const T kPriusCarLength{4.6257};  // in meters.
  const T kPriusCarWidth{1.8208};   // in meters.
  const T kPriusCarHeight{1.3957};  // in meters.
  const Translation3<T> kPriusCarToChassisTranslation(1.40948, 0., 0.69785);
  const Quaternion<T> kPriusCarToChassisRotation{Quaternion<T>::Identity()};

  // Registers a source for the given scene graph.
  const SourceId source_id = scene_graph->RegisterSource(frame_root);
  // Registers the Prius car frame.
  const GeometryFrame car_frame(
      detail::ResolveFrameName(frame_root, "car_frame"),
      initial_world_to_car_transform);
  const FrameId car_frame_id =
      scene_graph->RegisterFrame(source_id, car_frame);

  // Registers the Prius car chassis frame.
  const GeometryFrame car_chassis_frame(
      detail::ResolveFrameName(frame_root, "car_chassis_frame"),
      kPriusCarToChassisTranslation * kPriusCarToChassisRotation);
  const FrameId car_chassis_frame_id = scene_graph->RegisterFrame(
      source_id, car_frame_id, car_chassis_frame);

  // Registers a bounding box geometry for the whole car with the
  // car chassis frame as its origin.
  auto car_bounding_box =
      std::make_unique<GeometryInstance>(
          Isometry3<T>::Identity(), std::make_unique<Box>(
              kPriusCarLength, kPriusCarWidth, kPriusCarHeight));

  geometry_ids->insert(scene_graph->RegisterGeometry(
      source_id, car_chassis_frame_id, std::move(car_bounding_box)));

  // Sets up a frame pose aggregator to deal with the
  // drake::systems::rendering::PoseVector to drake::geometry::FramePoseVector
  // conversions.
  auto frame_pose_aggregator =
      builder->template AddSystem<FramePoseAggregator<T>>(source_id);

  // Fixes the car-to-chassis transform.
  auto fixed_chassis_pose =
      builder->template AddSystem<ConstantVectorSource<T>>(
          PoseVector<T>(kPriusCarToChassisRotation,
                        kPriusCarToChassisTranslation));

  builder->Connect(fixed_chassis_pose->get_output_port(),
                   frame_pose_aggregator->DeclareInput(car_chassis_frame_id));

  builder->Connect(frame_pose_aggregator->get_output_port(0),
                   scene_graph->get_source_pose_port(source_id));

  return frame_pose_aggregator->DeclareInput(car_frame_id);
}

}  // namespace delphyne
