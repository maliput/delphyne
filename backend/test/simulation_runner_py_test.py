#!/usr/bin/env python

import unittest
from simulation_runner_py import (
    AutomotiveSimulator,
    SimpleCarState,
    SimulatorRunner
)


class TestSimulationRunnerPy(unittest.TestCase):
    """
    Test units for the simulator_runner python bindings.
    """

    def __init__(self, *args, **kwargs):
        """Setups variables and objects common across all tests."""
        # Call parent class' init method.
        super(TestSimulationRunnerPy, self).__init__(*args, **kwargs)
        # Initialize class variables.
        self.simulator = AutomotiveSimulator()
        self.state = SimpleCarState()
        self.simulator.AddPriusSimpleCar("0", "DRIVING_TEST", self.state)
        self.simulation_step = 0.001
        self.callback_called = False

    def setUp(self):
        """Initializes variables before running each test case."""
        # Initialize callback flag.
        self.callback_called = False

    def callback_test(self):
        """Sets a flag to True."""
        self.callback_called = True

    def test_callback_when_paused(self):
        """Creates a simulator runner in paused mode and runs a simulation step,
        verifying that the registered python callback haven't been called.
        """
        # Creates a simulator runner that starts in paused mode.
        start_paused = True
        runner = SimulatorRunner(
            self.simulator, self.simulation_step, start_paused)
        # Register a step callback.
        runner.AddStepCallback(self.callback_test)
        # Starts the simulator runner.
        runner.Start()
        # Run a simulation step.
        runner.RunSimulationStep()

        # Checks that callback hasn't been called.
        self.assertFalse(self.callback_called)

    def test_callback_when_not_paused(self):
        """Creates a simulator runner and runs a simulation step,
        verifying that the registered python callback is called.
        """
        # Creates a simulator runner.
        runner = SimulatorRunner(self.simulator, self.simulation_step)
        # Register a step callback.
        runner.AddStepCallback(self.callback_test)

        # Checks that callback hasn't been called
        # before running a simulation step.
        self.assertFalse(self.callback_called)

        # Run a simulation step.
        runner.RunSimulationStep()

        # Checks that callback has been called
        # after a simulation step.
        self.assertTrue(self.callback_called)


if __name__ == '__main__':
    unittest.main()
