#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2022, Woven Planet. All rights reserved.
# Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

##############################################################################
# Documentation
##############################################################################

"""A group of common functions useful for python-scripted simulations"""

##############################################################################
# Imports
##############################################################################

import os

from . import agents

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


def get_delphyne_resource(path):
    """Resolve the path against delphyne resources root location."""
    for root in get_delphyne_resource_root().split(':'):
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
