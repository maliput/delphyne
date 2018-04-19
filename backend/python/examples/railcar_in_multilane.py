#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
An example of a couple of railcars running around in a closed-loop maliput
road.
"""

from __future__ import print_function

from pydrake.automotive import LaneDirection

from delphyne.bindings import (
    Any,
    AutomotiveSimulator,
    MaliputRailcarState,
    MaliputRailcarParams,
    RoadBuilder,
    SimulatorRunner
)

from delphyne.simulation_utils import (
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


def setup_railcar(simulator, name, road, lane):
    """Create a railcar by loading LoadableMaliputRailCar agent. The new
    car will have the provided name as an id and will start traversing the
    provided lane"""
    railcar_state = MaliputRailcarState()
    railcar_state.s = 0.0
    railcar_state.speed = 10.0

    lane_direction = LaneDirection(lane, True)

    start_params = MaliputRailcarParams()
    start_params.r = 0
    start_params.h = 0

    params = {
        "road": Any(road),
        "initial_with_s": Any(True),
        "lane_direction": Any(lane_direction),
        "start_params": Any(start_params)
    }

    simulator.AddLoadableCar("LoadableMaliputRailCar",
                             params,
                             name,
                             railcar_state)


def main():
    """
    Parses the command line options and launches the loadable-agent demo based
    on the provided configuration.
    """

    simulator = AutomotiveSimulator()

    builder = RoadBuilder(simulator)

    filename = "./install/share/delphyne/road_samples/multilane_sample.yaml"

    road = builder.AddMultilaneFromFile(filename)

    # Setup railcar 1
    lane_1 = road.junction(1).segment(0).lane(0)
    setup_railcar(simulator, "Railcar1", road, lane_1)

    # Setup railcar 2
    lane_2 = road.junction(1).segment(0).lane(1)
    setup_railcar(simulator, "Railcar2", road, lane_2)

    # Setup railcar 3
    lane_3 = road.junction(1).segment(0).lane(2)
    setup_railcar(simulator, "Railcar3", road, lane_3)

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
