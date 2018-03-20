#!/usr/bin/env python2.7

"""
This example shows how to run a simulation that includes a road. For the
time being two road examples are supported: dragway and onramp. To launch this
demo with the default values do:

```
$ road_loading.py
```

Or explicitly pass the desired road:

```
$ road_loading.py --road='dragway'
```

```
$ road_loading.py --road='onramp'
```

"""

#
# Copyright 2017 Toyota Research Institute
#

from __future__ import print_function

import argparse

from python_bindings import (
    RoadBuilder,
    SimulatorRunner
)
from simulation_utils import (
    build_simple_car_simulator,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP = 0.001


def main():
    """Simple demo that shows how to add a road to a simulation. Reads the
    road to be added from the command line args"""

    available_roads = ["dragway", "onramp"]

    parser = argparse.ArgumentParser(
        prog="roads",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-r", "--road",
                        default=available_roads[0],
                        const=available_roads[0],
                        nargs='?',
                        choices=available_roads,
                        help='The road to display')

    args = parser.parse_args()

    road = args.road

    simulator = build_simple_car_simulator()

    builder = RoadBuilder(simulator)

    if road == "dragway":
        # Add a dragway with 3 lanes and 100 meters long. Each lane is 3.7
        # meters wide and the road shoulder is 1 meter on each side. Finally
        # the maximum height of the road is 5 meters.
        builder.AddDragway("Demo dragway", 3, 100.0, 3.7, 1.0, 5.0)
    elif road == "onramp":
        builder.AddOnramp()
    else:
        raise RuntimeError("Option {} not recognized".format(road))

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
