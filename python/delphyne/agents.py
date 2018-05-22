#!/usr/bin/env python2.7
#
# Copyright 2018 Toyota Research Institute
#

#############################################################################
# Documentation
#############################################################################

"""
Agent utilities.
"""

from delphyne.agent_bindings2 import (
    TrajectoryAgent
)

#############################################################################
# Methods
#############################################################################

def add_trajectory_agent(simulator, robot_id, road, times, headings, translations):
    """
    Instantiates a trajectory agent with a trajectory defined by times, headings
    and translations.

    Args:
        simulator: the automotive simulator object
        robot_id: name of the agent
        road: maliput road geometry
        times: list of times defining the trajectory (floats)
        headings: list of yaw headings defining the trajectory (floats)
        translations: list of translations defining the trajectory (x, y, z)

    An example translations argument:
        translation = [[0.0, 0.0, 0.0], [1.25, 0.0, 0.0]]
    """
    print("Instantiating a trajectory agent")
    trajectory_agent = TrajectoryAgent()
    print("Instantiated!")
#     # Initial State
#     unused_initial_state = SimpleCarState()
#
#     # Parameters
#     parameters = {
#         "road": Any(road),
#         "times": Any(times),
#         "headings": Any(headings),
#         "translations": Any(translations)
#     }
#
#     # Instantiate
#     simulator.AddLoadableAgent("trajectory-agent",
#                                parameters,
#                                str(robot_id),
#                                unused_initial_state
#                                )

