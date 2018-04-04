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
    SimpleCarState,
)
from delphyne.launcher import Launcher


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
        if simulator_runner.IsRunning():
            simulator_runner.Stop()
        print("Simulation ended")
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


def build_simple_car_simulator(initial_positions=None):
    """Creates an AutomotiveSimulator instance and attachs a simple car to it.
    Returns the newly created simulator.
    """
    if initial_positions is None:
        initial_positions = [(0.0, 0.0)]
    simulator = AutomotiveSimulator()
    car_id = 0
    for car_position in initial_positions:
        state = SimpleCarState()
        state.y = car_position[0]
        state.x = car_position[1]
        driving_command = "DRIVING_COMMAND_" + str(car_id)
        simulator.AddPriusSimpleCar(str(car_id), driving_command, state)
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


def launch_visualizer(launcher, layout_filename):
    """Launches the project's visualizer with a given layout"""
    ign_visualizer = "visualizer"
    delphyne_resource_root = get_from_env_or_fail('DELPHYNE_RESOURCE_ROOT')
    layout_key = "--layout="
    layout_path = os.path.join(delphyne_resource_root, layout_filename)
    teleop_config = layout_key + layout_path
    launcher.launch([ign_visualizer, teleop_config])
