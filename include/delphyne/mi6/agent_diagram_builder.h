// Copyright 2018 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/framework_common.h>  // OutputPortIndex
#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/output_port.h>
#include <drake/systems/primitives/pass_through.h>
#include <drake/systems/rendering/pose_bundle.h>

#include "delphyne/macros.h"
#include "delphyne/mi6/diagram_bundle.h"
#include "delphyne/types.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/
/// @brief A builder that creates diagrams with inputs and outputs for agents.
///
/// This provides convenience methods and ensures that the resulting
/// diagrams has the requisite input and output ports. The final
/// deliverable is a @ref delphyne::DiagramBundle<T> "DiagramBundle".
///
///                         -----------
///                        |           | --> State
///      Traffic Poses --> |  Diagram  | --> Pose
///                        |           | --> Velocity
///                         -----------
///
template <typename T>
class AgentDiagramBuilder : public drake::systems::DiagramBuilder<T> {
 public:
  /// @brief Default constructor.
  /// @param[in] name : Convenient descriptive name for the diagram.
  explicit AgentDiagramBuilder(const std::string& name) : name_(name) {
    traffic_poses_ = this->AddSystem(std::make_unique<drake::systems::PassThrough<double>>(
        drake::Value<drake::systems::rendering::PoseBundle<double>>(0)));
    inputs_mapping_["traffic_poses"] = this->ExportInput(traffic_poses_->get_input_port());
  }

  /// @brief Export the specified state output port.
  ///
  /// Note that this may only be called once before building, since
  /// the contract for this builder is to create a diagram with one
  /// and only one state output port.
  ///
  /// @exception std::runtime_error if this method is called more
  /// than once before building.
  void ExportStateOutput(const drake::systems::OutputPort<T>& output) {
    DELPHYNE_VALIDATE(outputs_mapping_.count("state") == 0, std::runtime_error,
                      "A state output port has already been exported and this diagram "
                      "enforces that there can be only one.");
    outputs_mapping_["state"] = this->ExportOutput(output);
  }

  /// @brief Export the specified pose output port.
  ///
  /// Note that this may only be called once before building, since
  /// the contract for this builder is to create a diagram with one
  /// and only one pose output port.
  ///
  /// @exception std::runtime_error if this method is called more
  /// than once before building.
  void ExportPoseOutput(const drake::systems::OutputPort<T>& output) {
    DELPHYNE_VALIDATE(outputs_mapping_.count("pose") == 0, std::runtime_error,
                      "A pose output port has already been exported and this diagram "
                      "enforces that there can be only one.");
    outputs_mapping_["pose"] = this->ExportOutput(output);
  }

  /// @brief Export the specified velocity output port.
  ///
  /// Note that this may only be called once before building, since
  /// the contract for this builder is to create a diagram with one
  /// and only one velocity output port.
  ///
  /// @exception std::runtime_error if this method is called more
  /// than once before building.
  void ExportVelocityOutput(const drake::systems::OutputPort<T>& output) {
    DELPHYNE_VALIDATE(outputs_mapping_.count("velocity") == 0, std::runtime_error,
                      "A velocity output port has already been exported and this diagram "
                      "enforces that there can be only one.");
    outputs_mapping_["velocity"] = this->ExportOutput(output);
  }

  /// @brief Connect the input traffic poses port to internal systems.
  ///
  /// This is necessary since there are often multiple internal
  /// systems that require the traffic poses coming from outside the
  /// diagram. This connects them to the pass through system that
  /// makes that diagram input port available to multiple systems.
  void ConnectTrafficPosesTo(const drake::systems::InputPort<T>& destination) {
    this->Connect(traffic_poses_->get_output_port(), destination);
  }

  /// @brief Build the diagram bundle and append the indices.
  std::unique_ptr<DiagramBundle<T>> Build() {
    // Check that all indices have been set
    //   inputs: "traffic_poses"
    //   outputs: "state", "pose", "velocity"
    DELPHYNE_VALIDATE(outputs_mapping_.count("state") != 0, std::runtime_error,
                      "A state output port has not been exported (see "
                      "AgentDiagramBuilder::ExportStateOutput)");
    DELPHYNE_VALIDATE(outputs_mapping_.count("pose") != 0, std::runtime_error,
                      "A pose output port has not been exported (see "
                      "AgentDiagramBuilder::ExportPoseOutput)");
    DELPHYNE_VALIDATE(outputs_mapping_.count("velocity") != 0, std::runtime_error,
                      "A velocity output port has not been exported (see "
                      "AgentDiagramBuilder::ExportPoseOutput)");
    auto diagram = std::make_unique<DiagramBundle<T>>(this);
    diagram->set_name(name_);
    diagram->set_input_names(inputs_mapping_);
    diagram->set_output_names(outputs_mapping_);
    return diagram;
  }

 private:
  const std::string name_;
  drake::systems::PassThrough<double>* traffic_poses_;
  std::map<std::string, drake::systems::InputPortIndex> inputs_mapping_{};
  std::map<std::string, drake::systems::OutputPortIndex> outputs_mapping_{};
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
