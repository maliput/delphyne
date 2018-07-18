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
import numpy as np

from delphyne.agents import SimpleCar
from delphyne.simulation import (
    AutomotiveSimulator,
    SimulatorRunner
)
from delphyne.utilities import (
    launch_interactive_simulation
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


def on_agent_collision(runner, simulator, agents_in_collision):
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
        print("{} and {} have crashed!.".format(
            agent1.name(), agent2.name()
        ))
        ctx = simulator.get_context(agent1)
        agent1_pose = agent1.get_pose(ctx)
        agent1_velocity = agent1.get_velocity(ctx)
        print("{} was going at {} m/s and hit {} at {}.".format(
            agent1.name(), np.linalg.norm(agent1_velocity[3:]),
            agent2.name(), agent1_pose.translation()
        ))
        ctx = simulator.get_context(agent2)
        agent2_pose = agent2.get_pose(ctx)
        agent2_velocity = agent2.get_velocity(ctx)
        print("{} was going at {} m/s and hit {} at {}.".format(
            agent2.name(), np.linalg.norm(agent2_velocity[3:]),
            agent1.name(), agent2_pose.translation()
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
        runner.add_collision_callback(
            lambda agents_in_collision: on_agent_collision(
                runner, runner.get_simulator(), agents_in_collision
            )
        )
        runner.enable_collisions()
        runner.start()
