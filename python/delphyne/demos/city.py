#!/usr/bin/env python2.7
#
# Copyright 2018 Toyota Research Institute
#
"""
The city demo.

"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import itertools
import math
import os.path
import random

import pydrake.maliput.all as maliput

import delphyne.maliput as maliput_helpers
import delphyne.simulation as simulation
import delphyne.utilities as utilities


from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


def get_all_junctions(road):
    """
    A generator to retrieve all junctions found in the given `road`.
    """
    # TODO(hidmic): Code below sporadically segfaults. Check junction
    #               count using RoadGeometry's num_junctions() method
    #               once it is exposed.
    for i in itertools.count(0):
        junction = road.junction(i)
        if junction is None:
            break
        yield junction


def get_all_segments(road):
    """
    A generator to retrieve all segments found in the given `road`.
    """
    for junction in get_all_junctions(road):
        # TODO(hidmic): Code below sporadically segfaults. Check for
        #               segment count using Junction's num_segments()
        #               method once it is exposed.
        for i in range(2):
            segment = junction.segment(i)
            if segment is None:
                break
            yield segment


def get_all_lanes(road):
    """
    A generator to retrieve all lanes found in the given `road`.
    """
    for segment in get_all_segments(road):
        # TODO(hidmic): Check for lane count using Segment's
        #               num_lanes() method once it is exposed.
        yield segment.lane(0)


def get_lane_length(lane):
    """
    Returns the given `lane` total length.
    """
    # TODO(hidmic): Just query the Lane directly when the length() method
    #               gets exposed through Python bindings.
    gpstart0 = lane.ToGeoPosition(maliput.LanePosition(s=0., r=0., h=0.))
    gpstart1 = lane.ToGeoPosition(maliput.LanePosition(s=0.1, r=0., h=0.))
    x, y, z = 10e6 * (gpstart1.xyz() - gpstart0.xyz()) + gpstart0.xyz()
    lpend = lane.ToLanePosition(maliput.GeoPosition(x, y, z),
                                maliput.GeoPosition(0., 0., 0.),
                                0.)
    return lpend.srh()[0]


def get_lane_heading(lane, s_position):
    """
    Returns the `lane` heading at the given `s_position`
    along the centerline (i.e. r = h = 0).
    """
    # TODO(hidmic): Just query the Lane directly when the GetOrientation()
    #               method gets exposed through Python bindings.
    s_position_behind = s_position - 0.1
    if s_position_behind < 0.:
        s_position, s_position_behind = \
            s_position_behind, s_position
    x0_position, y0_position, _ = lane.ToGeoPosition(
        maliput.LanePosition(s=s_position_behind, r=0., h=0.)
    ).xyz()
    x1_position, y1_position, _ = lane.ToGeoPosition(
        maliput.LanePosition(s=s_position, r=0., h=0.)
    ).xyz()
    return math.atan2(y1_position - y0_position,
                      x1_position - x0_position)


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "City traffic",
        """
An example of city traffic with a variable number of railcars and
MOBIL controlled cars running in a closed-loop maliput road.
        """
    )
    parser.add_argument(
        "-n", "--num-rail-cars", default=40, type=int,
        help="The number of rails cars on scene (default: 40)."
    )
    parser.add_argument(
        "-m", "--num-mobil-cars", default=10, type=int,
        help="The number of MOBIL cars on scene (default: 10)."
    )

    return parser.parse_args()

##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    simulator = simulation.AutomotiveSimulator()

    filename = "{0}/roads/little_city.yaml".format(
        utilities.get_delphyne_resource_root())

    if not os.path.isfile(filename):
        print("Required file {} not found."
              " Please, make sure to install the latest delphyne-gui."
              .format(os.path.abspath(filename)))
        quit()

    # The road geometry
    road = simulator.set_road_geometry(
        maliput_helpers.create_multilane_from_file(
            file_path=filename
        )
    )

    # Gets all lanes in the road.
    lanes = (road.junction(i).segment(j).lane(k)
             for i in range(road.num_junctions())
             for j in range(road.junction(i).num_segments())
             for k in range(road.junction(i).segment(j).num_lanes()))

    # Determines all available spots for car positioning.
    car_distance = 6.0  # m
    lane_positions = [
        (lane, maliput.LanePosition(s=i * car_distance, r=0., h=0.))
        for lane in lanes for i in range(int(lane.length() / car_distance))
    ]

    # Ensures there's at least one spot for each car.
    maximum_car_count = len(lane_positions)
    total_car_count = args.num_rail_cars + args.num_mobil_cars
    if total_car_count > maximum_car_count:
        print(("No room for so many cars!"
               " Maximum car count is {}").format(
                   maximum_car_count))
        quit()

    # Allocates a lane position for each car in a pseudo-random fashion.
    random.seed(23)
    car_lane_positions = random.sample(lane_positions, total_car_count)
    rail_car_lane_positions = car_lane_positions[:args.num_rail_cars]
    mobil_car_lane_positions = car_lane_positions[args.num_rail_cars:]

    # Sets up all railcars.
    railcar_speed = 5.0  # (m/s)
    for n in range(args.num_rail_cars):
        lane, lane_position = rail_car_lane_positions[n]
        utilities.add_rail_car(
            simulator,
            name='rail{}'.format(n),
            lane=lane,
            position=lane_position.srh()[0],
            offset=0.0,  # m
            speed=railcar_speed,
            road_geometry=road)

    # Sets up all MOBIL cars.
    mobilcar_speed = 4.0  # (m/s)
    for m in range(args.num_mobil_cars):
        lane, lane_position = mobil_car_lane_positions[m]
        geo_position = lane.ToGeoPosition(lane_position)
        geo_orientation = lane.GetOrientation(lane_position)
        x_position, y_position, _ = geo_position.xyz()
        heading = geo_orientation.rpy().yaw_angle()

        utilities.add_mobil_car(
            simulator, name="mobil" + str(m),
            scene_x=x_position,
            scene_y=y_position,
            heading=heading,
            speed=mobilcar_speed,
            road_geometry=road)

    runner = simulation.SimulatorRunner(
        simulator=simulator,
        time_step=0.01,  # (secs)
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
