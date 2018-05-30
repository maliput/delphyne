#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#
#
# Documentation
#

"""A group of common functions useful for python-scripted simulations"""

#
# Imports
#

from __future__ import print_function

import os
import sys
import time

from contextlib import contextmanager
from delphyne.bindings import (
    AutomotiveSimulator,
)
from delphyne.agents import (
    MobilCar,
    RailCar,
    SimpleCar,
    TrajectoryAgent
)
from delphyne.launcher import Launcher

#
# Methods
#


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
    except RuntimeError as error_msg:
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
    agent = SimpleCar(str(robot_id),
                      position_x,  # scene x-coordinate (m)
                      position_y,  # scene y-coordinate (m)
                      0.0,         # heading (radians)
                      0.0)         # speed in the direction of travel (m/s)
    simulator.AddAgent(agent)

# pylint: disable=too-many-arguments
def add_mobil_car(simulator, name, x=0, y=0, heading=0.0, speed=1.0):
    """Instantiates a new MOBIL Car and adds
    it to the simulation.
    """
    agent = MobilCar(name,      # unique name
                     True,      # direction_of_travel
                     x,         # scene x-coordinate (m)
                     y,         # scene y-coordinate (m)
                     heading,   # heading (radians)
                     speed)     # speed in the s-direction (m/s)
    simulator.AddAgent(agent)

# pylint: disable=too-many-arguments
def add_rail_car(simulator, name, lane, position, offset, speed):
    """Instantiates a Railcar and adds it to the simulation.
    """
    # Note: keyword arguments not permitted with the bindings
    # TODO(daniel.stonier) true?
    agent = RailCar(name,      # unique name
                    lane,      # lane
                    True,      # direction_of_travel
                    position,  # lane s-coordinate (m)
                    offset,    # lane r-coordinate (m)
                    speed,     # speed in the s-direction (m/s)
                    5.0)       # nominal_speed (m/s)
    simulator.AddAgent(agent)

def add_trajectory_agent(simulator, robot_id, road, times, headings, waypoints):
    """
    Instantiates a trajectory agent with a trajectory defined by times,
    headings and translations.
    Args:
        simulator: the automotive simulator object
        robot_id: name of the agent
        times: list of times defining the trajectory (floats)
        headings: list of yaw headings defining the trajectory (floats)
        waypoints: list of points (x,y,z) defining the trajectory (x, y, z)
    An example translations argument:
        waypoints = [[0.0, 0.0, 0.0], [1.25, 0.0, 0.0]]
    """
    agent = TrajectoryAgent(str(robot_id),
                            times,       # timings (sec)
                            headings,    # list of headings (radians)
                            waypoints)   # list of x-y-z-tuples (m, m, m)
    simulator.AddAgent(agent)
