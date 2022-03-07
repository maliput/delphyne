// Copyright 2022 Toyota Research Institute
#pragma once

#include <memory>

#include <drake/systems/framework/event_status.h>
#include <drake/systems/framework/leaf_system.h>
#include <maliput/api/road_network.h>

#include "delphyne/macros.h"

#include "backend/dynamic_environment_handler.h"

namespace delphyne {

class DynamicEnvironmentHandlerSystem : public drake::systems::LeafSystem<double> {
 public:
  DynamicEnvironmentHandlerSystem(std::unique_ptr<DynamicEnvironmentHandler> deh) : deh_(std::move(deh)) {
    DELPHYNE_VALIDATE(deh_ != nullptr, std::invalid_argument, "Invalid DynamicEnvironmentHandler.");
    this->DeclarePerStepUnrestrictedUpdateEvent(&DynamicEnvironmentHandlerSystem::UpdatePhases);
  }

  /// Default destructor.
  ~DynamicEnvironmentHandlerSystem() override {}

 private:
  drake::systems::EventStatus UpdatePhases(const drake::systems::Context<double>& context,
                                           drake::systems::State<double>*) const {
    deh_->Update(context.get_time());
    return drake::systems::EventStatus::Succeeded();
  }

  mutable std::unique_ptr<DynamicEnvironmentHandler> deh_;
};

}  // namespace delphyne
