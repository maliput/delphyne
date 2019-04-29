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

KNOWN_ROADS = {
    'LineSingleLane': {
        'description': 'Single line lane of 100m length',
        'file_path': 'odr/SingleLane.xodr',
        'lane_id': '1_0_-1',
        'lane_position': 0.,
        'moving_forward': True,
    },
    'ArcSingleLane': {
        'description': ('Single arc lane of 100m length '
                        'and 40m of radius'),
        'file_path': 'odr/ArcLane.xodr',
        'lane_id': '1_0_1',
        'lane_position': 0.,
        'moving_forward': True,
    },
    'Roundabout': {
        'description': ('Single lane roundabout of 200m '
                        'length and ~31.83m of radius'),
        'lane_id': '1_0_1',
        'lane_position': 0.,
        'moving_forward': True,
    },
    'LShapeSection': {
        'description': ('Single road with 3 lane sections '
                        'with line-arc-line geometry'),
        'lane_id': '1_0_1',
        'lane_position': 0.,
        'moving_forward': True,
    },
    'LShapeRoad': {
        'description': ('3 roads connected each with line, '
                        'arc and line geometry respectively'),
        'lane_id': '1_0_1',
        'lane_position': 0.,
        'moving_forward': True,
    },
    'TShapeRoad': {
        'description': 'T intersection road with double hand roads',
        'lane_id': '1_0_1',
        'lane_position': 0.,
        'moving_forward': True,
    },
    'Crossing8Course': {
        'description': 'Crossing with 8 shape',
        'lane_id': '514_0_-1',
        'lane_position': 0.,
        'moving_forward': True,
    },
    'RRFigure8': {
        'description': 'Crossing with 8 shape (another)',
        'lane_id': '4_0_-1',
        'lane_position': 80.,
        'moving_forward': True,
    },
    'RRLongRoad': {
        'description': 'Long road with turning lanes',
        'lane_id': '3_0_-1',
        'lane_position': 16.,
        'moving_forward': False,
    },
}


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        'Mali Racing!',
        """
An example of a railcar running in an OpenDrive based maliput road.
        """
    )

    argument_help = """\
The OpenDRIVE road description to drive on. Either a path
to an .xodr file or one of the following well known roads: {}.
All relative paths are resolved against MALIDRIVE_RESOURCE_ROOT if not
found in the current working directory.
""".format(', '.join(KNOWN_ROADS))

    parser.add_argument(
        '-n', '--road-name', default='Roundabout', help=argument_help
    )
    return parser.parse_args()


def get_malidrive_resource(path):
    """Resolve the path against malidrive resources root location."""
    root = utilities.get_from_env_or_fail('MALIDRIVE_RESOURCE_ROOT')
    for root in root.split(':'):
        resolved_path = os.path.join(root, 'resources', path)
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

    if os.path.isfile(args.road_name):
        road = {
            'description': 'Custom user-provided road',
            'file_path': args.road_name,
            'lane_position': 0.,
            'moving_forward': True,
        }
    elif args.road_name in KNOWN_ROADS:
        road = KNOWN_ROADS[args.road_name]
        if 'file_path' not in road:
            road['file_path'] = os.path.join(
                'odr', args.road_name + '.xodr'
            )
        if not os.path.isabs(road['file_path']):
            road['file_path'] = get_malidrive_resource(road['file_path'])
    else:
        print("Unknown road {}.".format(args.road_name))
        quit()

    features = maliput.ObjFeatures()
    features.draw_arrows = True
    features.draw_elevation_bounds = False
    features.draw_stripes = True
    features.draw_lane_haze = False
    features.draw_branch_points = False

    # The road network
    road_network = builder.set_road_network(
        maliput.create_malidrive_from_file(
            name=os.path.splitext(
                os.path.basename(road['file_path'])
            )[0], file_path=road['file_path']
        ), features
    )

    # Find a lane
    road_geometry = road_network.road_geometry()
    if 'lane_id' in road:
        lane = road_geometry.ById().GetLane(road['lane_id'])
    else:
        lane = road_geometry.junction(0).segment(0).lane(0)

    # Setup a car
    railcar_speed = 15.0  # (m/s)
    utilities.add_rail_car(
        builder,
        name='car',
        lane=lane,
        position=road['lane_position'],
        offset=0.0,
        speed=railcar_speed,
        direction_of_travel=road['moving_forward']
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
