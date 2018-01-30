#!/usr/bin/env python

"""Unit tests for the simulation_runner python binding"""

# Copyright 2018 Open Source Robotics Foundation
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

import unittest
from simulation_runner_py import (
    AutomotiveSimulator,
    SimpleCarState,
    SimulatorRunner
)


class TestSimulationRunnerPy(unittest.TestCase):
    """
    Unit tests for the simulation_runner python binding
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
