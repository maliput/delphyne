#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#
"""
Load a simulation with one of a few sample maliput road networks.
"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import os
import sys

import delphyne.maliput as maliput
import delphyne.simulation as simulation
import delphyne.utilities as utilities

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "Maliput Roads",
        """
Load one of the various types of maliput road networks
into an empty (free of agents) simulation. For the time
being the following road network types are supported:
dragway, onramp, monolane and multilane.

This demo uses the subcommand style, where each road
type can handle different parameters. To get help on each
road type's parameters, run for example:

$ {0} multilane --help

Some examples:

$ {0} dragway --length=200 --shoulder-width=2.5
$ {0} onramp
$ {0} monolane
--filename='./install/share/delphyne/roads/double_ring.yaml'
$ {0} multilane
--filename='./install/share/delphyne/roads/circuit.yaml'
        """.format(os.path.basename(sys.argv[0]))
    )
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

    # Multilane subcommand
    multilane_parser = subparsers.add_parser("multilane")
    multilane_parser.add_argument("--filename",
                                  help="multilane file path",
                                  required=True)
    return parser.parse_args()


##############################################################################
# Main
##############################################################################

def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    simulator = simulation.AutomotiveSimulator()

    if args.road_type == "dragway":
        simulator.set_road_geometry(
            maliput.create_dragway(
                name="Demo Dragway",
                num_lanes=args.lanes,
                length=args.length,
                lane_width=args.lane_width,
                shoulder_width=args.shoulder_width,
                maximum_height=args.max_height
            )
        )
    elif args.road_type == "onramp":
        simulator.set_road_geometry(maliput.create_on_ramp())
    elif args.road_type in ["monolane", "multilane"]:
        try:
            if args.road_type == "monolane":
                simulator.set_road_geometry(
                    maliput.create_monolane_from_file(
                        file_path=args.filename
                    )
                )
            else:
                simulator.set_road_geometry(
                    maliput.create_multilane_from_file(
                        file_path=args.filename
                    )
                )
        except RuntimeError, error:
            print("There was an error trying to load the road network:")
            print(str(error))
            print("Exiting the simulation")
            sys.exit()
    else:
        raise RuntimeError("Option {} not recognized".format(args.road_type))

    runner = simulation.SimulatorRunner(
        simulator=simulator,
        time_step=0.001,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused,
        log=args.log
    )

    with utilities.launch_interactive_simulation(runner) as launcher:
        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            print("Running simulation for {0} seconds.".format(
                args.duration))
            runner.run_async_for(args.duration, launcher.terminate)
