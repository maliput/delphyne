#!/usr/bin/env python2.7
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

from __future__ import print_function

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
        "Gazoo Racing!",
        """
An example of three railcars and a variable number of MOBIL controlled
cars running in a closed-loop maliput road.

See also https://toyotagazooracing.com/
        """
    )
    parser.add_argument("-n", "--num-cars", default=3, type=int,
                        help="The number of MOBIL cars on scene (default: 3).")

    return parser.parse_args()

##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    if args.num_cars > 6 or args.num_cars < 0:
        print("The number of cars must be in the range of 0 to 6.")
        quit()

    mobil_cars_num = args.num_cars

    simulator = simulation.AutomotiveSimulator()

    filename = "{0}/roads/circuit.yaml".format(
        utilities.get_delphyne_resource_root())

    if not os.path.isfile(filename):
        print("Required file {} not found."
              " Please, make sure to install the latest delphyne-gui."
              .format(os.path.abspath(filename)))
        quit()

    # The road geometry
    road_geometry = simulator.set_road_geometry(
        maliput.create_multilane_from_file(
            file_path=filename
        )
    )

    # Setup railcar 1
    railcar_speed = 4.0  # (m/s)
    railcar_s = 0.0      # (m)
    robot_id = 1
    lane_1 = road_geometry.junction(2).segment(0).lane(0)
    utilities.add_rail_car(
        simulator,
        name=str(robot_id),
        lane=lane_1,
        position=railcar_s,
        offset=0.0,
        speed=railcar_speed,
        road_geometry=road_geometry)

    # Setup railcar 2
    railcar_speed = 8.0  # (m/s)
    railcar_s = 0.0      # (m)
    robot_id += 1
    lane_2 = road_geometry.junction(2).segment(0).lane(1)
    utilities.add_rail_car(
        simulator,
        name=str(robot_id),
        lane=lane_2,
        position=railcar_s,
        offset=0.0,
        speed=railcar_speed,
        road_geometry=road_geometry)

    # Setup railcar 3
    railcar_speed = 7.0  # (m/s)
    railcar_s = 0.0      # (m)
    robot_id += 1
    lane_3 = road_geometry.junction(2).segment(0).lane(2)
    utilities.add_rail_car(
        simulator,
        name=str(robot_id),
        lane=lane_3,
        position=railcar_s,
        offset=0.0,
        speed=railcar_speed,
        road_geometry=road_geometry)

    # Setup MOBIL cars.
    for i in range(mobil_cars_num):
        x_offset = 5.0       # (m)
        y_offset = 5.0       # (m)
        velocity_base = 2.0  # (m/s)
        robot_id += 1
        utilities.add_mobil_car(
            simulator=simulator,
            name=str(robot_id),
            scene_x=-10.0 + x_offset * (1 + i / 3),
            scene_y=0.0 + y_offset * (i % 3),
            heading=0.0,
            speed=velocity_base * i,
            road_geometry=road_geometry)

    runner = simulation.SimulatorRunner(
        simulator=simulator,
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
