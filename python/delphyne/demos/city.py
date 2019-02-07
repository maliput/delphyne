#!/usr/bin/env python3
#
# Copyright 2018 Toyota Research Institute
#
"""
The city demo.

"""
##############################################################################
# Imports
##############################################################################

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

    # The simulation builder
    builder = simulation.AgentSimulationBuilder()

    filename = utilities.get_delphyne_resource(
        "roads/little_city.yaml"
    )

    if not os.path.isfile(filename):
        print("Required file {} not found."
              " Please, make sure to install the latest delphyne-gui."
              .format(os.path.abspath(filename)))
        quit()

    # The road geometry
    road = builder.set_road_geometry(
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
            builder,
            name='rail{}'.format(n),
            lane=lane,
            position=lane_position.srh()[0],
            offset=0.0,  # m
            speed=railcar_speed
        )

    # Sets up all MOBIL cars.
    mobilcar_speed = 4.0  # (m/s)
    for m in range(args.num_mobil_cars):
        lane, lane_position = mobil_car_lane_positions[m]
        geo_position = lane.ToGeoPosition(lane_position)
        geo_orientation = lane.GetOrientation(lane_position)
        x_position, y_position, _ = geo_position.xyz()
        heading = geo_orientation.rpy().yaw_angle()

        utilities.add_mobil_car(
            builder,
            name="mobil" + str(m),
            scene_x=x_position,
            scene_y=y_position,
            heading=heading,
            speed=mobilcar_speed,
        )

    runner = simulation.SimulationRunner(
        simulation=builder.build(),
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
