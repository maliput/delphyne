#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
This example shows how to run a simulation that dynamically loads a car agent.
For the time being two car types are supported:


- simple, which just places a `LoadablePriusSimpleCarDouble` in an empty world.
- mobil, which places a `LoadableMobilControlledSimpleCarDouble` at the start
of a 200 meters dragway.


These examples can be executed by doing:

```
$ loadable_agent_simulation.py --type="simple"
```

and

```
$ loadable_agent_simulation.py --type="mobil"

```
"""

from __future__ import print_function

import argparse

from delphyne import (
    Any,
    AutomotiveSimulator,
    RoadBuilder,
    SimpleCarState,
    SimulatorRunner
)
from simulation_utils import (
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


def main():
    """
    Parses the command line options and launches the loadable-agent demo based
    on the provided configuration.
    """

    car_agent_types = ["simple", "mobil"]

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
        simulator.AddLoadableCar("LoadablePriusSimpleCar", {}, "0", state)

    elif args.type == "mobil":
        builder = RoadBuilder(simulator)

        dragway = builder.AddDragway("Demo dragway", 3, 200.0, 3.7, 3.0, 5.0)

        state = SimpleCarState()
        state.x = 0.0
        state.y = -3.7

        mobil_params = {
            "initial_with_s": Any(True),
            "road": Any(dragway)
        }

        # Instantiate a LoadableMobilControlledSimpleCar with "MOBIL0" as its
        # name and originally placed in (0.0, -3.7). The road for the car to
        # follow is the previously created dragway.
        simulator.AddLoadableCar("LoadableMobilControlledSimpleCar",
                                 mobil_params,
                                 "MOBIL0",
                                 state)
    else:
        raise RuntimeError("Option {} not recognized".format(args.type))

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
