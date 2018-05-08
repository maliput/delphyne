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
    Any,
    AutomotiveSimulator,
    MaliputRailcarParams,
    MaliputRailcarState,
    RoadBuilder,
    SimulatorRunner
)
from delphyne.simulation_utils import (
    launch_interactive_simulation
)
from pydrake.automotive import (
    LaneDirection,
    SimpleCarState
)

SIMULATION_TIME_STEP_SECS = 0.001

def add_simple_car(simulator):
    """Instanciates a new Simple Prius Car
    and adds it to the simulation.
    """
    # Creates the initial car state for the simple car.
    simple_car_state = SimpleCarState()
    simple_car_state.set_x(0.0)
    simple_car_state.set_y(3.7)
    # Instantiates a Loadable Simple Car
    simulator.AddLoadableAgent("simple-car", {}, "SimpleCar", simple_car_state)


def add_mobil_car(simulator, road):
    """Instanciates a new MOBIL Car and adds
    it to the simulation.
    """
    # Creates the initial car state for the MOBIL car.
    mobil_car_state = SimpleCarState()
    mobil_car_state.set_x(0.0)
    mobil_car_state.set_y(-3.7)
    mobil_params = {
        "initial_with_s": Any(True),
        "road": Any(road)
    }
    # Instantiates a Loadable MOBIL Car.
    simulator.AddLoadableAgent(
        "mobil-car", mobil_params, "MobilCar", mobil_car_state)


def add_maliput_railcar(simulator, road):
    """Instanciates a new Maliput Railcar and adds
    it to the simulation.
    """
    # Defines the lane that will be used by the dragway car.
    lane = road.junction(0).segment(0).lane(1)
    # Creates the initial car state for the Railcar.
    maliput_car_state = MaliputRailcarState()
    maliput_car_state.s = 0.0
    maliput_car_state.speed = 3.0
    lane_direction = LaneDirection(lane, True)
    start_params = MaliputRailcarParams()
    start_params.r = 0
    start_params.h = 0
    railcar_params = {
        "road": Any(road),
        "initial_with_s": Any(True),
        "lane_direction": Any(lane_direction),
        "start_params": Any(start_params)
    }
    # Instantiate a Loadable Rail Car.
    simulator.AddLoadableAgent("rail-car",
                               railcar_params,
                               "RailCar",
                               maliput_car_state)


def main():
    """Excercises a simulator loaded it with multiple different cars."""

    simulator = AutomotiveSimulator()
    builder = RoadBuilder(simulator)

    # Generates a dragway road.
    num_dragway_lanes = 3
    dragway_length = 100.0  # meters
    dragway_lane_width = 3.7  # meters
    dragway_shoulder_width = 3.0  # meters
    maximum_height = 5.0  # meters
    dragway = builder.AddDragway("Automotive Demo Dragway", num_dragway_lanes,
                                 dragway_length, dragway_lane_width,
                                 dragway_shoulder_width, maximum_height)

    # Adds the different cars.
    add_simple_car(simulator)
    add_mobil_car(simulator, dragway)
    add_maliput_railcar(simulator, dragway)

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    # Starts the simulation and runs it indefinitely.
    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
