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

from delphyne.bindings import (
    AutomotiveSimulator,
    RoadBuilder,
    SimulatorRunner
)
from delphyne.simulation_utils import (
    add_simple_car,
    add_maliput_railcar,
    add_mobil_car,
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

    car_id = 0
    if args.type == "simple":
        position_x = 0.0
        position_y = 1.0
        add_simple_car(simulator, car_id, position_x, position_y)

    elif args.type == "mobil":
        dragway = build_demo_dragway(simulator, "Mobil dragway")
        position_x = 0.0
        position_y = -3.7
        add_mobil_car(simulator, car_id, dragway,
                      position_x, position_y)

    elif args.type == "rail":
        dragway = build_demo_dragway(simulator, "Railcar dragway")

        # Instantiate a LoadableMaliputRailCar and
        # originally placed at the start of the second lane of the dragway
        # previously created. The car initial speed is 3 meters per sec.
        s_coordinate = 0.0
        speed = 3.0
        add_maliput_railcar(simulator, car_id, dragway, s_coordinate, speed)
    else:
        raise RuntimeError("Option {} not recognized".format(args.type))

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
