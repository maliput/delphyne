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

from python_bindings import (
    RoadBuilder,
    SimulatorRunner,
    AutomotiveSimulator
)
from simulation_utils import launch_visualizer

SIMULATION_TIME_STEP = 0.001


def main():
    """Simple demo that shows how to add a road to a simulation. Reads the
    road to be added from the command line args"""

    parser = argparse.ArgumentParser(
        prog="roads",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    subparsers = parser.add_subparsers(dest="road_type")

    # Dragway subcommand
    dragway_parser = subparsers.add_parser("dragway")
    dragway_parser.add_argument("--lanes", default=3,
                                type=int,
                                help="The number of lanes the dragway has")
    dragway_parser.add_argument("--length", default=100.0,
                                type=float,
                                help="The length of the dragway, in meters")
    dragway_parser.add_argument("--lane-width", default=3.7,
                                type=float,
                                help="The width of each lane, in meters")
    dragway_parser.add_argument("--shoulder-width", default=1.0,
                                type=float,
                                help="The width of the road shoulder, \
                                in meters")
    dragway_parser.add_argument("--max-height", default=5.0,
                                type=float,
                                help="The maximum allowed height for the road\
                                , in meters")

    # Onramp subcommand
    subparsers.add_parser("onramp")

    # Monolane subcommand
    monolane_parser = subparsers.add_parser("monolane")
    monolane_parser.add_argument("--filename",
                                 help="Monolane file path",
                                 required=True)

    args = parser.parse_args()

    road_type = args.road_type

    launcher = Launcher()

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
        builder.LoadMonolane(args.filename)
    else:
        raise RuntimeError("Option {} not recognized".format(road_type))

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
