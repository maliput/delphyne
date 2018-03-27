#!/usr/bin/env python2.7

"""
This example shows how to run a simulation that includes a road. For the
time being three road examples are supported: dragway, onramp and monolane.
This demo uses the subcommand style, where each road type can handle different
parameters (to list the available arguments just do
`$ road_loading_example <road_type> -h`). Below are some examples of usage:


A dragway that is 200 meters long and has a side-shoulder of 2.5 meters:

```
$ road_loading_example.py dragway --length=200 --shoulder-width=2.5
```

An on-ramp road:

```
$ road_loading_example.py onramp
```

Load an arbitrary monolane file:

```
$ road_loading_example.py monolane
--filename='./install/share/delphyne/road_samples/double_ring.yaml'
```

"""

#
# Copyright 2017 Toyota Research Institute
#

from __future__ import print_function

import argparse
import sys

from delphyne import (
    AutomotiveSimulator,
    RoadBuilder,
    SimulatorRunner
)
from simulation_utils import launch_interactive_simulation

SIMULATION_TIME_STEP_SECS = 0.001


def main():
    """Simple demo that shows how to add a road to a simulation. Reads the
    road to be added from the command line args"""

    parser = argparse.ArgumentParser(
        prog="road_loading",
        description="Simple demo that shows how to load a road network in a \
        simulation by using the RoadBuilder class.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    subparsers = parser.add_subparsers(dest="road_type")

    # Dragway subcommand
    dragway_parser = subparsers.add_parser("dragway")
    dragway_parser.add_argument("--lanes", default=3,
                                type=int,
                                help="the number of lanes the dragway has")
    dragway_parser.add_argument("--length", default=100.0,
                                type=float,
                                help="the length of the dragway, in meters")
    dragway_parser.add_argument("--lane-width", default=3.7,
                                type=float,
                                help="the width of each lane, in meters")
    dragway_parser.add_argument("--shoulder-width", default=1.0,
                                type=float,
                                help="the width of the road shoulder,\
                                in meters")
    dragway_parser.add_argument("--max-height", default=5.0,
                                type=float,
                                help="the maximum allowed height for the road,\
                                in meters")

    # Onramp subcommand
    subparsers.add_parser("onramp")

    # Monolane subcommand
    monolane_parser = subparsers.add_parser("monolane")
    monolane_parser.add_argument("--filename",
                                 help="monolane file path",
                                 required=True)

    args = parser.parse_args()

    road_type = args.road_type

    simulator = AutomotiveSimulator()

    builder = RoadBuilder(simulator)

    if road_type == "dragway":
        builder.AddDragway("Demo dragway",
                           args.lanes,
                           args.length,
                           args.lane_width,
                           args.shoulder_width,
                           args.max_height)
    elif road_type == "onramp":
        builder.AddOnramp()
    elif road_type == "monolane":
        try:
            builder.AddMonolaneFromFile(args.filename)
        except RuntimeError, error:
            print("There was an error trying to load the monolane file:")
            print(str(error))
            print("Exiting the simulation")
            sys.exit()
    else:
        raise RuntimeError("Option {} not recognized".format(road_type))

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
