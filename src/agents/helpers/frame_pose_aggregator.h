// Copyright 2018 Toyota Research Institute

#pragma once

#include <vector>

#include <drake/common/drake_copyable.h>
#include <drake/geometry/frame_kinematics_vector.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/input_port_descriptor.h>
#include <drake/systems/framework/leaf_system.h>

namespace delphyne {

/// A system that aggregates drake::systems::rendering::PoseVector inputs
/// into a single drake::geometry::FramePoseVector output, each one associated
/// to a specific frame ID. Akin in functionality to
/// drake::systems::rendering::PoseAggregator.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class FramePoseAggregator : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FramePoseAggregator)

  /// Constructs an aggregator that uses the given @p source_id.
  explicit FramePoseAggregator(const drake::geometry::SourceId& source_id);

  /// Declares a pose input port, associated with the given @p frame_id.
  /// @return The input drake::systems::rendering::PoseVector port
  /// descriptor.
  const drake::systems::InputPortDescriptor<T>& DeclareInput(
      const drake::geometry::FrameId& frame_id);

 private:
  // Returns a new drake::geometry::FramePoseVector instance to
  // populate.
  drake::geometry::FramePoseVector<T> MakeFramePoseVector() const;

  // Builds the outgoing drake::geometry::FramePoseVector based on
  // declared drake::systems::rendering::PoseVector inputs.
  void CalcFramePoseVector(const drake::systems::Context<T>& context,
                           drake::geometry::FramePoseVector<T>* output) const;

  // Aggregator's source ID to tag the outgoing drake::geometry::FramePoseVector
  // instances (@see MakeFramePoseVector()).
  const drake::geometry::SourceId source_id_{};
  // Aggregator inputs' associated frame IDs.
  std::vector<drake::geometry::FrameId> frame_ids_{};
};

}  // namespace delphyne
