#!/usr/bin/env python2.7

"""A group of common functions useful for the delphyne project"""

# Copyright 2017 Open Source Robotics Foundation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from pydrake.common import AddResourceSearchPath

from simulation_runner_py import (
    AutomotiveSimulator,
    SimpleCarState,
)


def add_drake_resource_path():
    """Adds the DRAKE_INSTALL_PATH environmental variable into
    drake's resource search path.
    """
    drake_install_path = get_from_env_or_fail('DRAKE_INSTALL_PATH')
    AddResourceSearchPath(os.path.join(drake_install_path, "share", "drake"))


def build_simple_car_simulator():
    """Creates an AutomotiveSimulator instance and attachs a simple car to it.
    Returns the newly created simulator.
    """
    simulator = AutomotiveSimulator()
    state = SimpleCarState()
    state.y = 0.0
    simulator.AddPriusSimpleCar("0", "DRIVING_COMMAND_0", state)
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


def launch_bridge(launcher):
    """Launches the duplex-ign-lcm-bridge"""
    lcm_ign_bridge = "duplex-ign-lcm-bridge"
    launcher.launch([lcm_ign_bridge, "1"])


def launch_visualizer(launcher, layout_filename):
    """Launches the project's visualizer with a given layout"""
    ign_visualizer = "visualizer"
    delphyne_ws_dir = get_from_env_or_fail('DELPHYNE_WS_DIR')
    teleop_config = os.path.join(delphyne_ws_dir,
                                 "install",
                                 "share",
                                 "delphyne",
                                 layout_filename)
    launcher.launch([ign_visualizer, teleop_config])
