#!/usr/bin/env python3
#
# Copyright 2021 Toyota Research Institute
#

"""Unit tests for the agent python binding"""

import time
import unittest
from delphyne.simulation import (
    AgentSimulationBuilder,
    SimulationRunner
)
from delphyne.agents import SimpleCarBlueprint


class TestAgentPy(unittest.TestCase):
    """
    Unit tests for the simulation_runner python binding
    """
    SIMULATION_STEP = 0.001

    def __init__(self, *args, **kwargs):
        """Setups variables and objects common across all tests."""
        # Call parent class' init method.
        super(TestAgentPy, self).__init__(*args, **kwargs)

        # Class variables.
        self.agent_name = "simple-car"

    def setUp(self):
        """Initializes variables before running each test case."""
        # Sets up a simulation.
        builder = AgentSimulationBuilder()
        # Adds a prius car to the simulation.
        builder.add_agent(SimpleCarBlueprint(
            self.agent_name, 10.0, -10.0, 0.0, 0.0
        ))
        # Build the simulation and binds it to the
        # simulation runner.
        self.runner = SimulationRunner(
            builder.build(), self.SIMULATION_STEP)

        # Starts the simulator runner.
        self.runner.start()

        # Waits until the simulator initializes its machinery.
        time.sleep(0.1)

        # Gets agent.
        self.agent = self.runner.get_simulation().get_agent_by_name("simple-car")

    def test_name_method(self):
        """
        Call name() method
        """
        # Checks agent's name.
        self.assertEqual(self.agent.name(), self.agent_name)

    def test_get_pose_translation_method(self):
        """
        Call get_pose_translation() method
        """

        translation = self.agent.get_pose_translation()
        # Checks agent's translation vector.
        self.assertEqual(translation[0], 10.)
        self.assertEqual(translation[1], -10.)
        self.assertEqual(translation[2], 0.)

    def test_get_pose_rotation_method(self):
        """
        Call get_pose_rotation() method
        """

        rotation = self.agent.get_pose_rotation()
        # Checks agent's rotation quaternion.
        self.assertEqual(rotation[0], 0.)
        self.assertEqual(rotation[1], 0.)
        self.assertEqual(rotation[2], 0.)
        self.assertEqual(rotation[3], 1.)


if __name__ == '__main__':
    unittest.main()
