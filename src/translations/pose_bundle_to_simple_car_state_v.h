// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/systems/framework/leaf_system.h>

#include "delphyne/protobuf/simple_car_state.pb.h"
#include "delphyne/protobuf/simple_car_state_v.pb.h"

namespace delphyne {

/// @brief A system that takes a PoseBundle and generates an array of
/// SimpleCarStates (SimpleCarState_V).
class PoseBundleToSimpleCarState_V : public drake::systems::LeafSystem<double> {
 public:
  PoseBundleToSimpleCarState_V();

  /// @brief Returns the output port of the system.
  const drake::systems::OutputPort<double>& get_simple_car_state_v_output()
      const {
    return this->get_output_port(output_port_index_);
  }

  /// @brief Returns the input port descriptor for the pose bundle. This port
  /// expects a drake::sytems::rendering::PoseBundle<double>.
  const drake::systems::InputPortDescriptor<double>&
  get_pose_bundle_input_port() const {
    return get_input_port(pose_bundle_input_port_index_);
  }

 protected:
  /// @brief Calculates the SimpleCarState_V message based on the given context.
  void CalcSimpleCarState_V(const drake::systems::Context<double>& context,
                            ignition::msgs::SimpleCarState_V* output) const;

 private:
  // @brief The pose bundle input port index assigned by the system.
  int pose_bundle_input_port_index_{};

  // @brief The abstract output port index assigned by the system.
  int output_port_index_{};
};

}  // namespace delphyne
