#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""A group of common functions useful for the delphyne project"""

import os

from python_bindings import (
    AutomotiveSimulator,
    SimpleCarState,
)


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
