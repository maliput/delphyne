#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
This demo consists of a suite of dynamically loaded cars,
running simultaneously on a dragway road.
For the time being, three cars are supported:
   - A Prius Simple Car.
   - A MOBIL Simple Car.
   - A MaliputRailCar.
"""

from __future__ import print_function

from delphyne.bindings import (
    AutomotiveSimulator,
    RoadBuilder,
    SimulatorRunner
)

from delphyne.simulation_utils import (
    add_simple_car,
    add_maliput_railcar,
    add_mobil_car,
    add_trajectory_agent,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


def main():
    """Excercises a simulator loaded with multiple different cars."""

    simulator = AutomotiveSimulator()
    builder = RoadBuilder(simulator)

    # A Dragway!
    num_dragway_lanes = 4
    dragway_length = 100.0  # meters
    dragway_lane_width = 3.7  # meters
    dragway_shoulder_width = 3.0  # meters
    maximum_height = 5.0  # meters
    dragway = builder.AddDragway("Automotive Demo Dragway", num_dragway_lanes,
                                 dragway_length, dragway_lane_width,
                                 dragway_shoulder_width, maximum_height)

    # Adds the different cars.
    simple_car_position_x = 0.0
    simple_car_position_y = 1.5 * 3.7
    car_id = 0
    add_simple_car(simulator, car_id, simple_car_position_x,
                   simple_car_position_y)

    mobil_car_position_x = 0.0
    mobil_car_position_y = -0.5 * 3.7
    car_id += 1
    add_mobil_car(simulator, car_id, dragway,
                  mobil_car_position_x, mobil_car_position_y)

    railcar_s = 0.0
    railcar_speed = 3.0
    car_id += 1
    lane = dragway.junction(0).segment(0).lane(1)

    add_maliput_railcar(simulator, car_id, dragway, lane, railcar_s, railcar_speed)

    car_id += 1
    times = [0.0, 5.0, 10.0, 15.0, 20.0]
    headings = [0.0, 0.0, 0.0, 0.0, 0.0]
    translations = [
        [0.0, -5.55, 0.0],
        [10.0, -5.55, 0.0],
        [30.0, -5.55, 0.0],
        [60.0, -5.55, 0.0],
        [100.0, -5.55, 0.0]
    ]

    add_trajectory_agent(simulator,
                         car_id,
                         times,
                         headings,
                         translations)

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    # Starts the simulation and runs it indefinitely.
    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
