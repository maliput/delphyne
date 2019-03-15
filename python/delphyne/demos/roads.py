#!/usr/bin/env python3
#
# Copyright 2017 Toyota Research Institute
#
"""
Load a simulation with one of a few sample maliput road networks.
"""
##############################################################################
# Imports
##############################################################################

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
dragway, onramp, and multilane.

This demo uses the subcommand style, where each road
type can handle different parameters. To get help on each
road type's parameters, run for example:

$ {0} multilane --help

Some examples:

$ {0} dragway --length=200 --shoulder-width=2.5
$ {0} onramp
$ {0} multilane
--filename='./install/share/delphyne/roads/circuit.yaml'
$ {0} malidrive
--filename='./install/share/delphyne/roads/circuit.xodr'
--name=my_circuit
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

    # Multilane subcommand
    multilane_parser = subparsers.add_parser("multilane")
    multilane_parser.add_argument("--filename",
                                  help="multilane file path",
                                  required=True)

    # Malidrive subcommand
    malidrive_parser = subparsers.add_parser("malidrive")
    malidrive_parser.add_argument("--filename",
                                  help="malidrive file path",
                                  required=True)
    malidrive_parser.add_argument("--name",
                                  help="malidrive road name",
                                  default="maliroad")

    return parser.parse_args()


##############################################################################
# Main
##############################################################################

def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    builder = simulation.AgentSimulationBuilder()

    if args.road_type == "dragway":
        builder.set_road_geometry(
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
        builder.set_road_geometry(maliput.create_on_ramp())
    elif args.road_type == "multilane":
        try:
            builder.set_road_geometry(
                maliput.create_multilane_from_file(
                    file_path=args.filename
                )
            )
        except RuntimeError as error:
            print("There was an error trying to load the road network:")
            print(str(error))
            print("Exiting the simulation")
            sys.exit()
    elif args.road_type == "malidrive":
        try:
            builder.set_road_geometry(
                maliput.create_malidrive_from_file(
                    name=args.name,
                    file_path=args.filename
                )
            )
        except RuntimeError as error:
            print("There was an error trying to load the road network:")
            print(str(error))
            print("Exiting the simulation")
            sys.exit()
    else:
        raise RuntimeError("Option {} not recognized".format(args.road_type))

    runner = simulation.SimulationRunner(
        simulation=builder.build(),
        time_step=0.001,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused,
        log=args.log,
        logfile_name=args.logfile_name
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
