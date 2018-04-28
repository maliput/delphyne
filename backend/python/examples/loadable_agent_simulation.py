#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
This example shows how to run a simulation that dynamically loads a car agent.
For the time being two car types are supported:


- simple, which just places a `LoadablePriusSimpleCarDouble` in an empty world.
- mobil, which places a `LoadableMobilControlledSimpleCarDouble` at the start
of a 100 meters dragway.
- rail, which places a `LoadableMaliputRailCar` in the center lane of a
three-lane dragway.


These examples can be executed by doing:

```
$ loadable_agent_simulation.py --type="simple"
```

```
$ loadable_agent_simulation.py --type="mobil"

```
```
$ loadable_agent_simulation.py --type="rail"

```

"""

from __future__ import print_function

import argparse

from pydrake.automotive import LaneDirection

from delphyne.bindings import (
    Any,
    AutomotiveSimulator,
    MaliputRailcarState,
    MaliputRailcarParams,
    SimpleCarState,
    RoadBuilder,
    SimulatorRunner
)
from delphyne.simulation_utils import (
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


def build_demo_dragway(simulator, name):
    """Build a demo dragway that has 3 lanes. The dragway is 100 meters long,
    each lane is 3.7 meters wide and the shoulder width is 3 meters. Finally,
    the maximum allowed height is 5 meters.
    """
    builder = RoadBuilder(simulator)
    return builder.AddDragway(name, 3, 100.0, 3.7, 3.0, 5.0)


def main():
    """
    Parses the command line options and launches the loadable-agent demo based
    on the provided configuration.
    """

    car_agent_types = ["simple", "mobil", "rail"]

    parser = argparse.ArgumentParser(
        prog="loadable_agent_simulation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-t", "--type", default=car_agent_types[0],
                        dest="type",
                        choices=car_agent_types,
                        help="The type of car agent to load")

    simulator = AutomotiveSimulator()

    args = parser.parse_args()

    if args.type == "simple":
        state = SimpleCarState()
        state.x = 0.0
        state.y = 1.0

        # Instantiate a LoadablePriusSimpleCar with 0 id and originally
        # placed in (0.0, 1.0)
        simulator.AddLoadableAgent("prius-simple-car",
                                   {},
                                   args.type,
                                   state
                                   )

    elif args.type == "mobil":
        dragway = build_demo_dragway(simulator, "Mobil dragway")

        state = SimpleCarState()
        state.x = 0.0
        state.y = -3.7

        mobil_params = {
            "initial_with_s": Any(True),
            "road": Any(dragway)
        }

        # Instantiate a MobilControlledSimpleCar
        # and originally placed in (0.0, -3.7). The road for the car to
        # follow is the previously created dragway.
        simulator.AddLoadableAgent("mobil-controlled-simple-car",
                                   mobil_params,
                                   args.type,
                                   state)
    elif args.type == "rail":
        dragway = build_demo_dragway(simulator, "Railcar dragway")

        lane = dragway.junction(0).segment(0).lane(1)

        state = MaliputRailcarState()
        state.s = 0.0
        state.speed = 3.0

        lane_direction = LaneDirection(lane, True)

        start_params = MaliputRailcarParams()
        start_params.r = 0
        start_params.h = 0

        railcar_params = {
            "road": Any(dragway),
            "initial_with_s": Any(True),
            "lane_direction": Any(lane_direction),
            "start_params": Any(start_params)
        }

        # Instantiate a LoadableMaliputRailCar and
        # originally placed at the start of the second lane of the dragway
        # previously created. The car initial speed is 3 meters per sec.
        simulator.AddLoadableAgent("maliput-rail-car",
                                   railcar_params,
                                   args.type,
                                   state)
    else:
        raise RuntimeError("Option {} not recognized".format(args.type))

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()