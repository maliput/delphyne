#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#
##############################################################################
# Documentation
##############################################################################

"""A group of common functions useful for python-scripted simulations"""

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import os
import sys
import time

from contextlib import contextmanager

from . import agents
from . import launcher

##############################################################################
# Launchers
##############################################################################


@contextmanager
def launch_interactive_simulation(simulator_runner,
                                  layout="layout_with_teleop.config"):
    """Defines a context manager function used to hande the execution of an
    interactive simulation. An interactive simulation launches the delphyne
    visualizer in a separate process and ends the simulation when the
    visualizer is closed."""

    launch_manager = launcher.Launcher()
    try:
        launch_visualizer(launch_manager, layout)
        yield launch_manager
        launch_manager.wait(float("Inf"))
    except RuntimeError as error_msg:
        sys.stderr.write("ERROR: {}".format(error_msg))
    finally:
        if simulator_runner.is_interactive_loop_running():
            simulator_runner.stop()
        print("Simulation ended. I'm happy, you should be too.")
        print_simulation_stats(simulator_runner)
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launch_manager.kill()


def launch_visualizer(launcher_manager, layout_filename):
    """Launches the project's visualizer with a given layout"""
    ign_visualizer = "visualizer"
    layout_key = "--layout="
    layout_path = os.path.join(get_delphyne_resource_root(),
                               "layouts", layout_filename)
    teleop_config = layout_key + layout_path
    launcher_manager.launch([ign_visualizer, teleop_config])

##############################################################################
# Environment
##############################################################################


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


##############################################################################
# Agents
##############################################################################
#
# These are methods that typically don't do much, but basically ensure that
# adding an agent to the simulation is pythonic and doesn't leave you with
# zombie objects after ownership has been transferred to a simulation
#
# - construct the agent
# - add it to the simulator
# - check that the operation succeeded
# -   failure : throw an exception
# -   success : return a handle to the agent
#
# TODO(daniel.stonier) exception handling and return handles to the agent

def add_simple_car(simulator, name, position_x, position_y):
    """Adds a simple car to the simulation."""
    agent = agents.SimpleCar(
        name=name,
        x=position_x,  # scene x-coordinate (m)
        y=position_y,  # scene y-coordinate (m)
        heading=0.0,   # heading (radians)
        speed=0.0)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)


# pylint: disable=too-many-arguments
def add_mobil_car(simulator, name, scene_x, scene_y,
                  heading, speed, road_geometry):
    """Adds a lane changing (MOBIL) car to the simulation."""
    agent = agents.MobilCar(
        name=name,                 # unique name
        direction_of_travel=True,  # with or against the lane s-direction
        x=scene_x,                 # scene x-coordinate (m)
        y=scene_y,                 # scene y-coordinate (m)
        heading=heading,           # heading (radians)
        speed=speed,               # the s-direction (m/s)
        road_geometry=road_geometry)  # maliput road geometry
    simulator.add_agent(agent)


# pylint: disable=too-many-arguments
def add_rail_car(simulator, name, lane, position, offset,
                 speed, road_geometry):
    """Adds a centre-line following rail car to the simulation."""
    agent = agents.RailCar(
        name=name,                       # unique name
        lane=lane,                       # lane
        direction_of_travel=True,        # direction_of_travel
        longitudinal_position=position,  # lane s-coordinate (m)
        lateral_offset=offset,           # lane r-coordinate (m)
        speed=speed,                     # initial speed in s-direction (m/s)
        nominal_speed=5.0,               # nominal_speed (m/s)
        road_geometry=road_geometry)     # maliput road geometry
    simulator.add_agent(agent)


# pylint: disable=too-many-arguments
def add_trajectory_agent(simulator, name, times, headings, waypoints):
    """
    Adds a trajectory agent to the simulation.
    The trajectory is a time parameterised curve that is constructed
    from lists of knot points defined by times, headings and translations.
    Args:
        simulator: the automotive simulator object
        name: name of the agent
        times: list of times defining the trajectory (floats)
        headings: list of yaw headings defining the trajectory (floats)
        waypoints: list of points (x,y,z) defining the trajectory (x, y, z)
    An example waypoints argument:
        waypoints = [[0.0, 0.0, 0.0], [1.25, 0.0, 0.0]]
    """
    agent = agents.TrajectoryAgent(
        name,
        times,       # timings (sec)
        headings,    # list of headings (radians)
        waypoints)   # list of x-y-z-tuples (m, m, m)
    simulator.add_agent(agent)

##############################################################################
# Other
##############################################################################


def print_simulation_stats(simulator_runner):
    """Get the interactive simulation statistics and print them on standard
    output.
    """
    stats = simulator_runner.get_stats()
    print("= Simulation stats ==========================")
    print("  Simulation runs: {}".format(stats.total_runs()))
    print("  Simulation steps: {}".format(stats.total_executed_steps()))
    print("  Elapsed simulation time: {}s".format(
        stats.total_elapsed_simtime()))
    print("  Elapsed real time: {}s".format(stats.total_elapsed_realtime()))
    print("=============================================")
