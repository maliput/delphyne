#!/usr/bin/env python3
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
def launch_interactive_simulation(simulation_runner,
                                  layout="layout_with_teleop.config",
                                  bare=False):
    """Defines a context manager function used to hande the execution of an
    interactive simulation. An interactive simulation launches the delphyne
    visualizer in a separate process and ends the simulation when the
    visualizer is closed."""

    launch_manager = launcher.Launcher()
    try:
        if not bare:
            launch_visualizer(launch_manager, layout)
        yield launch_manager
        launch_manager.wait(float("Inf"))
    except RuntimeError as error_msg:
        sys.stderr.write("ERROR: {}".format(error_msg))
    finally:
        if simulation_runner.is_logging():
            print("Simulation has been logged in {}".format(
                simulation_runner.get_log_filename()))
            simulation_runner.stop_logging()
        if simulation_runner.is_interactive_loop_running():
            simulation_runner.stop()
        print("Simulation ended. I'm happy, you should be too.")
        print_simulation_stats(simulation_runner)
        # This is needed to avoid a possible deadlock. See
        # SimulatorRunner class description.
        time.sleep(0.5)
        launch_manager.kill()


def launch_visualizer(launcher_manager, layout_filename=None,
                      plugin_injection=None, bundle_path=None):
    """
    Launches the project's visualizer with a given layout and using the
    given bundled package, if any.
    """
    ign_visualizer = "visualizer"
    ign_visualizer_args = []
    if layout_filename:
        layout_key = "--layout="
        layout_path = get_delphyne_resource(
            os.path.join("layouts", layout_filename)
        )
        ign_visualizer_args.append(layout_key + layout_path)
    if plugin_injection:
        ign_visualizer_args.append("--inject-plugin=" + plugin_injection)
    if bundle_path:
        ign_visualizer_args.append("--package=" + bundle_path)
    launcher_manager.launch([ign_visualizer] + ign_visualizer_args)

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


def get_delphyne_gui_resource_root():
    """Return the root path where delphyne-gui resources live"""
    return get_from_env_or_fail('DELPHYNE_GUI_RESOURCE_ROOT')


def get_delphyne_resource(path):
    """Resolve the path against delphyne resources root location."""
    for root in get_delphyne_resource_root().split(':'):
        resolved_path = os.path.join(root, path)
        if os.path.exists(resolved_path):
            return resolved_path
    return ''


def get_delphyne_gui_resource(path):
    """Resolve the path against delphyne resources root location."""
    for root in get_delphyne_gui_resource_root().split(':'):
        resolved_path = os.path.join(root, path)
        if os.path.exists(resolved_path):
            return resolved_path
    return ''


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


# pylint: disable=too-many-arguments
def add_simple_car(builder, name, position_x, position_y, heading=0.0,
                   speed=0.0):
    """
    Adds a simple car to the simulation and returns its blueprint.
    """
    return builder.add_agent(
        agents.SimpleCarBlueprint(
            name=name,
            x=position_x,  # scene x-coordinate (m)
            y=position_y,  # scene y-coordinate (m)
            heading=heading,   # heading (radians)
            speed=speed      # speed in the direction of travel (m/s)
        )
    )


# pylint: disable=too-many-arguments
def add_mobil_car(builder, name, scene_x, scene_y, heading, speed):
    """
    Adds a lane changing (MOBIL) car to the simulation and returns
    its blueprint.
    """
    return builder.add_agent(
        agents.MobilCarBlueprint(
            name=name,                 # unique name
            direction_of_travel=True,  # with or against the lane s-direction
            x=scene_x,                 # scene x-coordinate (m)
            y=scene_y,                 # scene y-coordinate (m)
            heading=heading,           # heading (radians)
            speed=speed                # the s-direction (m/s)
        )
    )


# pylint: disable=too-many-arguments
def add_rail_car(builder, name, lane, position, offset,
                 speed, direction_of_travel=True):
    """
    Adds a centre-line following rail car to the simulation and
    returns its blueprint.
    """
    return builder.add_agent(
        agents.RailCarBlueprint(
            name=name,                                # unique name
            lane=lane,                                # lane
            direction_of_travel=direction_of_travel,  # direction_of_travel
            longitudinal_position=position,           # lane s-coordinate (m)
            lateral_offset=offset,                    # lane r-coordinate (m)
            speed=speed,                              # initial speed in
                                                      # s-direction (m/s)
            nominal_speed=20.0                        # nominal_speed (m/s)
        )
    )


# pylint: disable=too-many-arguments
def add_trajectory_agent(builder, name, times, headings, waypoints):
    """
    Adds a trajectory agent to the simulation and returns its blueprint.
    The trajectory is a time parameterised curve that is constructed
    from lists of knot points defined by times, headings and translations.
    Args:
        builder: the agent simulation builder object
        name: name of the agent
        times: list of times defining the trajectory (floats)
        headings: list of yaw headings defining the trajectory (floats)
        waypoints: list of points (x,y,z) defining the trajectory (x, y, z)
    An example waypoints argument:
        waypoints = [[0.0, 0.0, 0.0], [1.25, 0.0, 0.0]]
    """
    return builder.add_agent(
        agents.TrajectoryAgentBlueprint(
            name,
            times,       # timings (sec)
            headings,    # list of headings (radians)
            waypoints   # list of x-y-z-tuples (m, m, m)
        )
    )

##############################################################################
# Other
##############################################################################


def emplace(method):
    """
    Python decorator that performs an in-place replacement of the given
    `method` using the decorated function, that takes the associated
    class object and the method itself as arguments, in that order.

    :param method: unbound class method.
    """
    def _do_emplace(method_decorator):
        cls = method.im_class
        method_wrapper = method_decorator(cls, method)
        setattr(cls, method.__name__, method_wrapper)
        return method_wrapper
    return _do_emplace


def print_simulation_stats(simulation_runner):
    """Get the interactive simulation statistics and print them on standard
    output.
    """
    stats = simulation_runner.get_stats()
    print("= Simulation stats ==========================")
    print("  Simulation runs: {}".format(stats.total_runs()))
    print("  Simulation steps: {}".format(stats.total_executed_steps()))
    print("  Elapsed simulation time: {}s".format(
        stats.total_elapsed_simtime()))
    print("  Elapsed real time: {}s".format(stats.total_elapsed_realtime()))
    print("=============================================")
