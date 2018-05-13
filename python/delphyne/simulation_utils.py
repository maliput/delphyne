#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""A group of common functions useful for python-scripted simulations"""

from __future__ import print_function

import os
import sys
import time

from contextlib import contextmanager
from delphyne.bindings import (
    AutomotiveSimulator,
    MaliputRailcarParams,
    MaliputRailcarState,
    MobilCarAgentParams,
    RailCarAgentParams
)
from delphyne.launcher import Launcher
from pydrake.automotive import (
    LaneDirection,
    SimpleCarState
)


@contextmanager
def launch_interactive_simulation(simulator_runner,
                                  layout="layoutWithTeleop.config"):
    """Defines a context manager function used to hande the execution of an
    interactive simulation. An interactive simulation launches the delphyne
    visualizer in a separate process and ends the simulation when the
    visualizer is closed."""

    launcher = Launcher()
    try:
        launch_visualizer(launcher, layout)
        yield launcher
        launcher.wait(float("Inf"))
    except RuntimeError, error_msg:
        sys.stderr.write("ERROR: {}".format(error_msg))
    finally:
        if simulator_runner.IsInteractiveLoopRunning():
            simulator_runner.Stop()
        print("Simulation ended")
        print_simulation_stats(simulator_runner)
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


def print_simulation_stats(simulator_runner):
    """Get the interactive simulation statistics and print them on standard
    output.
    """
    stats = simulator_runner.get_stats()
    print("= Simulation stats ==========================")
    print("  Simulation runs: {}".format(stats.TotalRuns()))
    print("  Simulation steps: {}".format(stats.TotalExecutedSteps()))
    print("  Elapsed simulation time: {}s".format(stats.TotalElapsedSimtime()))
    print("  Elapsed real time: {}s".format(stats.TotalElapsedRealtime()))
    print("=============================================")


def build_simple_car_simulator(initial_positions=None):
    """Creates an AutomotiveSimulator instance and attachs a simple car to it.
    Returns the newly created simulator.
    """
    if initial_positions is None:
        initial_positions = [(0.0, 0.0)]
    simulator = AutomotiveSimulator()
    car_id = 0
    for car_position in initial_positions:
        add_simple_car(simulator, car_id, car_position[1], car_position[0])
        car_id += 1
    return simulator


def get_from_env_or_fail(var):
    """Retrieves an env variable for a given name, fails if not found."""
    value = os.environ.get(var)
    if value is None:
        raise RuntimeError("{} is not in the environment, did you remember to"
                           "source setup.bash?\n".format(var))

    # Since it is an environment variable, the very end may have a colon;
    # strip it here
    return value.rstrip(':')


def get_delphyne_resource_root():
    """Return the root path where delphyne resources live"""
    return get_from_env_or_fail('DELPHYNE_RESOURCE_ROOT')


def launch_visualizer(launcher, layout_filename):
    """Launches the project's visualizer with a given layout"""
    ign_visualizer = "visualizer"
    layout_key = "--layout="
    layout_path = os.path.join(get_delphyne_resource_root(), layout_filename)
    teleop_config = layout_key + layout_path
    launcher.launch([ign_visualizer, teleop_config])


def add_simple_car(simulator, robot_id, position_x=0, position_y=0):
    """Instantiates a new Simple Prius Car
    and adds it to the simulation.
    """
    # Initial State
    simple_car_state = SimpleCarState()
    simple_car_state.set_x(position_x)
    simple_car_state.set_y(position_y)

    # Instantiate
    simulator.AddLoadableAgent(
        "simple-car", str(robot_id), simple_car_state, None)


def add_mobil_car(simulator, robot_id, road, position_x=0, position_y=0):
    """Instantiates a new MOBIL Car and adds
    it to the simulation.
    """
    # Initial State
    mobil_car_state = SimpleCarState()
    mobil_car_state.set_x(position_x)
    mobil_car_state.set_y(position_y)

    # Parameters
    agent_params = MobilCarAgentParams(True)

    # Instantiate
    simulator.AddLoadableAgent("mobil-car",
                               str(robot_id),
                               mobil_car_state,
                               road,
                               agent_params)


def add_maliput_railcar(simulator, robot_id, road, s_coordinate=0, speed=0):
    """Instantiates a new Maliput Railcar and adds
    it to the simulation.
    """
    # Maliput
    lane = road.junction(0).segment(0).lane(1)

    # Initial State
    railcar_state = MaliputRailcarState()
    railcar_state.s = s_coordinate
    railcar_state.speed = speed
    lane_direction = LaneDirection(lane, True)

    # Parameters
    start_params = MaliputRailcarParams()
    start_params.r = 0
    start_params.h = 0
    agent_params = RailCarAgentParams(lane_direction, start_params)

    # Instantiate
    simulator.AddLoadableAgent("rail-car",
                               str(robot_id),
                               railcar_state,
                               road,
                               agent_params)

def add_trajectory_agent(simulator, robot_id, road):
    """Instantiates a new Maliput Railcar and adds
    it to the simulation.
    """
    # Maliput
    lane = road.junction(0).segment(0).lane(1)
    lane_direction = LaneDirection(lane, True)

    # Initial State
    unused_initial_state = SimpleCarState()

    # Parameters
    parameters = {
        "road": Any(road),
    }

    # Instantiate
    simulator.AddLoadableAgent("trajectory-agent",
                               parameters,
                               str(robot_id),
                               unused_initial_state
                               )