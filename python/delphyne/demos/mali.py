#!/usr/bin/env python3
#
# Copyright 2017 Toyota Research Institute
#
"""
The gazoo demo.
```
"""
##############################################################################
# Imports
##############################################################################

import os.path

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
        "Mali Racing!",
        """
An example of a railcar running in an OpenDrive based maliput road.
        """
    )
    parser.add_argument("--road-file", required=True,
                        help="The OpenDrive network to drive on.")

    return parser.parse_args()

##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    builder = simulation.AgentSimulationBuilder()

    if not os.path.isfile(args.road_file):
        print("Required file {} not found."
              .format(os.path.abspath(args.road_file)))
        quit()

    features = maliput.ObjFeatures()
    features.draw_arrows = True
    features.draw_elevation_bounds = False
    features.draw_stripes = True
    features.draw_lane_haze = False
    features.draw_branch_points = False

    # The road geometry
    road_geometry = builder.set_road_geometry(
        maliput.create_malidrive_from_file(
            name="mali-road",
            file_path=args.road_file
        ), features
    )

    # Setup railcar 1
    railcar_speed = 20.0  # (m/s)
    railcar_s = 0.0      # (m)
    robot_id = 1
    lane_1 = road_geometry.junction(0).segment(0).lane(0)
    utilities.add_rail_car(
        builder,
        name=str(robot_id),
        lane=lane_1,
        position=railcar_s,
        offset=0.0,
        speed=railcar_speed
    )

    runner = simulation.SimulationRunner(
        simulation=builder.build(),
        time_step=0.015,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused,
        log=args.log,
        logfile_name=args.logfile_name)

    with utilities.launch_interactive_simulation(runner) as launcher:
        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            print("Running simulation for {0} seconds.".format(
                args.duration))
            runner.run_async_for(args.duration, launcher.terminate)