#!/usr/bin/env python2.7
#
# Copyright 2018 Toyota Research Institute
#
"""
The crash demo.
"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import math
from functools import wraps

from delphyne.agents import SimpleCar
from delphyne.simulation import (
    AutomotiveSimulator,
    SimulatorRunner
)
from delphyne.utilities import (
    launch_interactive_simulation,
    emplace
)


from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


def parse_arguments():
    """Argument passing and demo documentation."""
    parser = helpers.create_argument_parser(
        "Car Crash!",
        """
An example that exercises collision detection by setting up multiple cars
in collision course.
        """
    )
    return parser.parse_args()


@emplace(SimulatorRunner.add_collision_callback)
def add_collision_callback(_, method):
    """
    Decorates :meth:`SimulatorRunner.add_collision_callback` method
    to handle collision callbacks that take a :class:`SimulatorRunner`
    instance and a list of tuples of :class:`AgentBase` instances that
    are in collision as arguments in that order.
    """
    @wraps(method)
    def _method_wrapper(self, callback):
        @wraps(callback)
        def _callback_wrapper(agents_in_collision):
            callback(self, agents_in_collision)
        return method(self, _callback_wrapper)
    return _method_wrapper


def on_agent_collision(runner, agents_in_collision):
    """
    Callback on for collision between agents in simulation,
    stopping the runner if *any* collision is detected.

    :param runner: Current simulation runner.
    :type runner: :class:`delphyne.simulation.SimulatorRunner`
    :param agents_in_collision: List of agents (e.g. cars) currently
                                in collision.
    :type agents_in_collision: list[tuple[:class:`delphyne.agents.AgentBase`,
                                          :class:`delphyne.agents.AgentBase`]]
    """
    print("Collisions have been detected!")
    for agent1, agent2 in agents_in_collision:
        print("{} and {} have crashed.".format(
            agent1.name(), agent2.name()
        ))
    print("Simulation stopped.")
    runner.stop()


##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    simulator = AutomotiveSimulator()

    agent = SimpleCar(
        name="racer0",
        x=0.0,  # scene x-coordinate (m)
        y=-50.0,  # scene y-coordinate (m)
        heading=math.pi/2,    # heading (radians)
        speed=5.0)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    agent = SimpleCar(
        name="racer1",
        x=-50.0,  # scene x-coordinate (m)
        y=0.0,     # scene y-coordinate (m)
        heading=0.0,    # heading (radians)
        speed=5.1)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    agent = SimpleCar(
        name="racer2",
        x=0.0,  # scene x-coordinate (m)
        y=50.0,  # scene y-coordinate (m)
        heading=-math.pi/2,    # heading (radians)
        speed=5.0)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    agent = SimpleCar(
        name="racer3",
        x=50.0,  # scene x-coordinate (m)
        y=0.0,     # scene y-coordinate (m)
        heading=math.pi,    # heading (radians)
        speed=5.1)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    runner = SimulatorRunner(simulator,
                             time_step=0.001,  # (secs)
                             realtime_rate=args.realtime_rate,
                             paused=args.paused,
                             log=args.log,
                             logfile_name=args.logfile_name)

    with launch_interactive_simulation(runner):
        # Adds a callback to check for agent collisions.
        runner.add_collision_callback(on_agent_collision)
        runner.enable_collisions()
        runner.start()
