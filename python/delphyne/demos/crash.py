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


def check_for_collisions(runner, simulator, agents):
    """
    Checks for collisions between agents in simulation,
    stopping the runner if *any* collision is detected.
    :param runner: Current simulation runner.
    :type runner: delphyne.simulation.SimulationRunner
    :param simulator: Current simulator.
    :type simulator: delphyne.simulation.AutomotiveSimulator
    """
    collisions = simulator.get_collisions()
    if isinstance(agents[0], SimpleCar):
        agents[0].print()

    if any(collisions):
        print("Collision detected between the following car IDs:")
        for pair in collisions:
            print(pair)
        print("Simulation stopped.")
        runner.stop()


##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    agents = []

    print("--------------- Startup ---------------")
    simulator = AutomotiveSimulator()

    agent = SimpleCar(
        name="racer0",
        x=0.0,  # scene x-coordinate (m)
        y=-50.0,  # scene y-coordinate (m)
        heading=math.pi/2,    # heading (radians)
        speed=5.0)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    print(agent)
    agents.append(agent)

    agent = SimpleCar(
        name="racer1",
        x=-50.0,  # scene x-coordinate (m)
        y=0.0,     # scene y-coordinate (m)
        heading=0.0,    # heading (radians)
        speed=5.1)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    print(agent)
    agents.append(agent)

    agent = SimpleCar(
        name="racer2",
        x=0.0,  # scene x-coordinate (m)
        y=50.0,  # scene y-coordinate (m)
        heading=-math.pi/2,    # heading (radians)
        speed=5.0)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    print(agent)
    agents.append(agent)

    agent = SimpleCar(
        name="racer3",
        x=50.0,  # scene x-coordinate (m)
        y=0.0,     # scene y-coordinate (m)
        heading=math.pi,    # heading (radians)
        speed=5.1)     # speed in the direction of travel (m/s)
    simulator.add_agent(agent)

    print(agent)
    agents.append(agent)

    print("--------------- End startup --------------")
    runner = SimulatorRunner(simulator,
                             time_step=0.001,  # (secs)
                             realtime_rate=args.realtime_rate,
                             paused=args.paused)

    with launch_interactive_simulation(runner):
        # Adds a callback to check for agent collisions.
        runner.add_step_callback(
            lambda: check_for_collisions(runner, simulator, agents)
        )

        runner.start()
