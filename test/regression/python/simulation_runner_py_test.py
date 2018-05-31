#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""Unit tests for the simulation_runner python binding"""

import time
import unittest
from delphyne.bindings import (
    AutomotiveSimulator,
    SimulatorRunner
)
from delphyne.agents import SimpleCar

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
        self.simulator = AutomotiveSimulator()
        self.runner = None
        self.callback_called = False

    def setUp(self):
        """Initializes variables before running each test case."""
        # Initialize callback flag.
        self.callback_called = False
        # Add a prius car to the simulation.
        agent = SimpleCar("simple-car", 0.0, 0.0, 0.0, 0.0)
        self.simulator.AddAgent(agent)
        # Creates a simulator runner.
        self.runner = SimulatorRunner(
            self.simulator, self.SIMULATION_STEP)
        # Register a step callback.
        self.runner.AddStepCallback(self.callback_test)

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
        self.runner.Start()

        self.runner.PauseSimulation()

        # Waits until the simulator initializes its machinery.
        time.sleep(0.1)

        # Ensure simulator is paused.
        self.assertTrue(self.runner.IsSimulationPaused())

        # Checks that callback has been called.
        self.assertTrue(self.callback_called)

    def test_callback_when_unpaused(self):
        """Creates a simulator runner and runs a simulation step,
        verifying that the registered python callback was called.
        """
        # Checks that callback hasn't been called.
        self.assertFalse(self.callback_called)

        # Starts the simulator runner.
        self.runner.Start()

        # Ensure simulator is not paused.
        self.assertFalse(self.runner.IsSimulationPaused())

        # Waits until the simulator initializes its machinery.
        time.sleep(0.1)

        # Checks that callback has been called.
        self.assertTrue(self.callback_called)


if __name__ == '__main__':
    unittest.main()
