#!/usr/bin/env python3
#
# Copyright 2017 Toyota Research Institute
#

"""Unit tests for the simulation_runner python binding"""

import time
import unittest
from delphyne.simulation import (
    AgentSimulationBuilder,
    SimulationRunner
)
from delphyne.agents import SimpleCarBlueprint


class TestSimulationRunnerPy(unittest.TestCase):
    """
    Unit tests for the simulation_runner python binding
    """
    SIMULATION_STEP = 0.001

    def __init__(self, *args, **kwargs):
        """Setups variables and objects common across all tests."""
        # Call parent class' init method.
        super(TestSimulationRunnerPy, self).__init__(*args, **kwargs)
        # Initialize class variables.
        self.runner = None
        self.callback_called = False

    def setUp(self):
        """Initializes variables before running each test case."""
        # Initialize callback flag.
        self.callback_called = False
        # Sets up a simulation.
        builder = AgentSimulationBuilder()
        # Adds a prius car to the simulation.
        builder.add_agent(SimpleCarBlueprint(
            "simple-car", 0.0, 0.0, 0.0, 0.0
        ))
        # Build the simulation and binds it to the
        # simulation runner.
        self.runner = SimulationRunner(
            builder.build(), self.SIMULATION_STEP)
        # Register a step callback.
        self.runner.add_step_callback(self.callback_test)

    def callback_test(self):
        """Sets a flag to True."""
        self.callback_called = True

    def test_callback_when_paused(self):
        """Creates a simulator runner and runs a simulation step,
        verifying that the registered python callback was called.
        """
        # Checks that callback hasn't been called.
        self.assertFalse(self.callback_called)

        # Starts the simulator runner.
        self.runner.start()

        self.runner.pause_simulation()

        # Waits until the simulator initializes its machinery.
        time.sleep(0.1)

        # Ensure simulator is paused.
        self.assertTrue(self.runner.is_simulation_paused())

        # Checks that callback has been called.
        self.assertTrue(self.callback_called)

    def test_callback_when_unpaused(self):
        """Creates a simulator runner and runs a simulation step,
        verifying that the registered python callback was called.
        """
        # Checks that callback hasn't been called.
        self.assertFalse(self.callback_called)

        # Starts the simulator runner.
        self.runner.start()

        # Ensure simulator is not paused.
        self.assertFalse(self.runner.is_simulation_paused())

        # Waits until the simulator initializes its machinery.
        time.sleep(0.1)

        # Checks that callback has been called.
        self.assertTrue(self.callback_called)


if __name__ == '__main__':
    unittest.main()
