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


def main():
    """
    Parses the command line options and launches the loadable-agent demo based
    on the provided configuration.
    """

    simulator = AutomotiveSimulator()

    builder = RoadBuilder(simulator)

    filename = "./install/share/delphyne/road_samples/multilane_sample.yaml"

    road = builder.AddMultilaneFromFile(filename)

    # TODO(basicNew): I had to C&P the three railcar instantiations due to
    # memory issues. Need to digg a bit on the cause.

    # Setup railcar 1
    lane_1 = road.junction(1).segment(0).lane(0)

    railcar_state_1 = MaliputRailcarState()
    railcar_state_1.s = 0.0
    railcar_state_1.speed = 10.0

    lane_direction_1 = LaneDirection(lane_1, True)

    start_params_1 = MaliputRailcarParams()
    start_params_1.r = 0
    start_params_1.h = 0

    railcar_params_1 = {
        "road": Any(road),
        "initial_with_s": Any(True),
        "lane_direction": Any(lane_direction_1),
        "start_params": Any(start_params_1)
    }

    simulator.AddLoadableCar("LoadableMaliputRailCar",
                             railcar_params_1,
                             "Railcar1",
                             railcar_state_1)

    # Setup railcar 2
    lane_2 = road.junction(1).segment(0).lane(1)

    railcar_state_2 = MaliputRailcarState()
    railcar_state_2.s = 0.0
    railcar_state_2.speed = 10.0

    lane_direction_2 = LaneDirection(lane_2, True)

    start_params_2 = MaliputRailcarParams()
    start_params_2.r = 0
    start_params_2.h = 0

    railcar_params_2 = {
        "road": Any(road),
        "initial_with_s": Any(True),
        "lane_direction": Any(lane_direction_2),
        "start_params": Any(start_params_2)
    }

    simulator.AddLoadableCar("LoadableMaliputRailCar",
                             railcar_params_2,
                             "Railcar2",
                             railcar_state_2)

    # Setup railcar 3
    lane_3 = road.junction(1).segment(0).lane(2)

    railcar_state_3 = MaliputRailcarState()
    railcar_state_3.s = 0.0
    railcar_state_3.speed = 10.0

    lane_direction_3 = LaneDirection(lane_3, True)

    start_params_3 = MaliputRailcarParams()
    start_params_3.r = 0
    start_params_3.h = 0

    railcar_params_3 = {
        "road": Any(road),
        "initial_with_s": Any(True),
        "lane_direction": Any(lane_direction_3),
        "start_params": Any(start_params_3)
    }

    simulator.AddLoadableCar("LoadableMaliputRailCar",
                             railcar_params_3,
                             "Railcar3",
                             railcar_state_3)

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
