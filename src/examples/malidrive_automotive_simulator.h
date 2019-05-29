// Copyright 2019 Toyota Research Institute
#pragma once

#include <memory>
#include <string>
#include <utility>

#include "gen/rail_follower_params.h"
#include "gen/rail_follower_state.h"
#include "systems/lane_direction.h"
#include "systems/rail_follower.h"
#include "maliput/api/road_network.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

#include "examples/railcar_logger_system.h"
#include <map>

namespace delphyne {

class MalidriveAutomotiveSimulator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MalidriveAutomotiveSimulator)

  MalidriveAutomotiveSimulator();
  ~MalidriveAutomotiveSimulator() = default;

  /// Sets `road_network` whose RoadGeometry will host car agents.
  ///
  /// This class will take ownership of `road_network`.
  /// @param road_network is the RoadNetwork to host.
  /// @pre `road_network` must not be nullptr.
  /// @return The bare pointer to the RoadNetwork that `road_network` holds.
  const maliput::api::RoadNetwork* SetRoadNetwork(
      std::unique_ptr<const maliput::api::RoadNetwork> road_network);

  /// Creates a MaliputRailcar with `name` name running at
  /// `initial_lane_direction` with `params` parameters and `initial_state`.
  ///
  /// When `add_logger`, it also creates a RailcarLoggerSystem attached to it
  /// so debugging information is printed.
  int AddPriusMaliputRailcar(
      const std::string& name,
      const LaneDirection& initial_lane_direction,
      const delphyne::RailFollowerParams<double>& params =
          delphyne::RailFollowerParams<double>(),
      const delphyne::RailFollowerState<double>& initial_state =
          delphyne::RailFollowerState<double>(),
      bool add_logger = true);

  /// Returns whether the automotive simulator has started.
  bool has_started() const { return simulator_ != nullptr; }

  void Start(double target_realtime_rate);

  void SetMaliputRailcarAccelerationCommand(int id, double acceleration);

  void StepBy(double time_step);

 private:
  void CheckNameUniqueness(const std::string& name);

  int allocate_vehicle_number();

  void BuildAndInitialize();

  void Build();

  void InitializeMaliputRailcars();

  int next_vehicle_number_{};
  std::map<const delphyne::RailFollower<double>*,
           std::pair<delphyne::RailFollowerParams<double>,
                     delphyne::RailFollowerState<double>>>
      railcar_configs_;
  std::map<const delphyne::RailFollower<double>*,
           const RailcarLoggerSystem*>
      railcar_loggers_;
  std::map<int, drake::systems::System<double>*> vehicles_;
  std::unique_ptr<const maliput::api::RoadNetwork> road_network_{};
  std::unique_ptr<drake::systems::DiagramBuilder<double>> builder_{};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_{};
  std::unique_ptr<drake::systems::Simulator<double>> simulator_{};
};

}  // namespace delphyne
