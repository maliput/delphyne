#!/usr/bin/env python3
#
# Copyright 2019 Toyota Research Institute
#
"""
The mali demo.
```
"""
##############################################################################
# Imports
##############################################################################

import os.path

import delphyne.maliput as maliput
import delphyne.simulation as simulation
import delphyne.utilities as utilities

from delphyne.utilities import launch_interactive_simulation

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        'Mali Racing!',
        """
An example of a railcar running in an OpenDrive based maliput road.
        """
    )
    parser.add_argument('-i', '--road-file', default='Roundabout.xodr',
                        help=('The OpenDrive network to drive on. '
                              'Resolve path against MALIDRIVE_RESOURCE_ROOT '
                              'if not found in the current working dir.'))
    return parser.parse_args()


def get_malidrive_resource(path):
    """Resolve the path against malidrive resources root location."""
    root = utilities.get_from_env_or_fail('MALIDRIVE_RESOURCE_ROOT')
    for root in root.split(':'):
        resolved_path = os.path.join(root, 'resources', 'odr', path)
        if os.path.exists(resolved_path):
            return resolved_path
    return ''


##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    builder = simulation.AgentSimulationBuilder()

    if not os.path.isfile(args.road_file):
        resolved_road_file = get_malidrive_resource(args.road_file)
        if not os.path.isfile(resolved_road_file):
            print("Required file {} not found."
                  .format(os.path.abspath(args.road_file)))
            quit()
        args.road_file = resolved_road_file

    features = maliput.ObjFeatures()
    features.draw_arrows = True
    features.draw_elevation_bounds = False
    features.draw_stripes = True
    features.draw_lane_haze = False
    features.draw_branch_points = False

    # The road network
    road_network = builder.set_road_network(
        maliput.create_malidrive_from_file(
            name="mali-road",
            file_path=args.road_file
        ), features
    )

    # Find a lane
    road_geometry = road_network.road_geometry()
    first_junction_found = road_geometry.junction(0)
    first_lane_found = first_junction_found.segment(0).lane(0)

    # Setup railcar
    railcar_speed = 20.0  # (m/s)
    railcar_s = 0.0      # (m)
    utilities.add_rail_car(
        builder,
        name='rail-car',
        lane=first_lane_found,
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

    with launch_interactive_simulation(runner, bare=args.bare) as launcher:
        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            print("Running simulation for {0} seconds.".format(
                args.duration))
            runner.run_async_for(args.duration, launcher.terminate)
