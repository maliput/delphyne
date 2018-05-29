#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
An example of three railcars and a variable number of MOBIL controlled car
running in a closed-loop maliput road.

In order to run the demo with a number of MOBIL cars different to three, a
number between 0 and 6 can be passed as an argument. For example:

```
maliput_racing_circuit.py --num-cars=2
```
"""
from __future__ import print_function

import argparse
import os.path

from delphyne.bindings import (
    AutomotiveSimulator,
    RoadBuilder,
    SimulatorRunner
)

from delphyne.simulation_utils import (
    add_rail_car,
    add_mobil_car,
    get_delphyne_resource_root,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.015


def main():
    """
    Parses the command line options and launches the loadable-agent demo based
    on the provided configuration.
    """
    parser = argparse.ArgumentParser(
        prog="maliput_racing_circuit",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-n", "--num-cars", default=3, type=int,
                        help="The number of MOBIL cars on scene.")

    args = parser.parse_args()

    if args.num_cars > 6 or args.num_cars < 0:
        print("The number of cars must be in the range of 0 to 6.")
        quit()

    mobil_cars_num = args.num_cars

    simulator = AutomotiveSimulator()

    builder = RoadBuilder(simulator)

    filename = "{0}/roads/circuit.yaml".format(
        get_delphyne_resource_root())

    if not os.path.isfile(filename):
        print("Required file {} not found."
              " Please, make sure to install the latest delphyne-gui."
              .format(os.path.abspath(filename)))
        quit()

    road = builder.AddMultilaneFromFile(filename)

    # Setup railcar 1
    railcar_speed = 4.0  # Units in m/s.
    railcar_s = 0.0  # Units in m.
    robot_id = 1
    lane_1 = road.junction(2).segment(0).lane(0)
    add_rail_car(simulator,
                 name=str(robot_id),
                 lane=lane_1,
                 position=railcar_s,
                 offset=0.0,
                 speed=railcar_speed
                )

    # Setup railcar 2
    railcar_speed = 8.0  # Units in m/s.
    railcar_s = 0.0  # Units in m.
    robot_id += 1
    lane_2 = road.junction(2).segment(0).lane(1)
    add_rail_car(simulator,
                 name=str(robot_id),
                 lane=lane_2,
                 position=railcar_s,
                 offset=0.0,
                 speed=railcar_speed)

    # Setup railcar 3
    railcar_speed = 7.0  # Units in m/s.
    railcar_s = 0.0  # Units in m.
    robot_id += 1
    lane_3 = road.junction(2).segment(0).lane(2)
    add_rail_car(simulator,
                 name=str(robot_id),
                 lane=lane_3,
                 position=railcar_s,
                 offset=0.0,
                 speed=railcar_speed)

    # Setup MOBIL cars.
    for i in range(mobil_cars_num):
        x_offset = 5.0  # Units in m.
        y_offset = 5.0  # Units in m.
        velocity_base = 2.0  # Units in m/s.
        mobil_position_x = -10.0 + x_offset * (1 + i / 3)
        mobil_position_y = .0 + y_offset * (i % 3)
        mobil_velocity = velocity_base * i
        robot_id += 1
        add_mobil_car(simulator, robot_id, road, mobil_position_x,
                      mobil_position_y, mobil_velocity)

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    simulation_duration = 80  # Units in s.
    with launch_interactive_simulation(runner) as launcher:
        print("Running simulation for {0} seconds.".format(
            simulation_duration))
        runner.RunAsyncFor(simulation_duration, launcher.terminate)


if __name__ == "__main__":
    main()
