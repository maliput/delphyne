#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""This is a minimal example of starting an automotive simulation using a
python binding to the C++ `SimulatorRunner` class.

Note that this is not a configurable demo, it will just create a sample
simulation with a prius car that can be driven around.

Check the other examples in this directory for more advanced uses.
"""

# pylint: disable=W0201

from __future__ import print_function

import argparse
import random
import time

from delphyne.bindings import SimulatorRunner
from delphyne.simulation_utils import (
    build_simple_car_simulator,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


class SimulationStats(object):
    """This is a simple class to keep statistics of the simulation, just
    averaging the time it takes to execute a simulation step from the outside
    world. Every 1000 measures, the values are printed to stdout.
    """

    def __init__(self):
        """Just init the stats"""
        self.reset()

    def reset(self):
        """Clear all values"""
        self._time_sum = 0.0
        self._samples_count = 0
        self._current_start_time = None

    def print_stats(self):
        """Print the stats"""
        print(
            "Average simulation step takes {delta}ms"
            .format(delta=(self._time_sum / self._samples_count) * 1000))

    def start(self):
        """Record the time when we start measuring"""
        self._current_start_time = time.time()

    def record_tick(self):
        """A simulation tick happened. Record it.
        Every 1000 ticks print the stats and reset
        """
        end_time = time.time()
        delta = end_time - self._current_start_time
        self._time_sum += delta
        self._samples_count += 1
        if self._samples_count == 1000:
            self.print_stats()
            self.reset()
        self.start()


def random_print():
    """Print a message at random, roughly every 500 calls"""
    if random.randint(1, 500) == 1:
        print("One in five hundred")


def main():
    """Spawn an automotive simulator making use of the python bindings"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--paused", action='store_true',
                        dest='start_paused',
                        default=False, help="Start simulator in paused mode")
    args = parser.parse_args()

    stats = SimulationStats()

    simulator = build_simple_car_simulator()

    runner = SimulatorRunner(simulator,
                             SIMULATION_TIME_STEP_SECS,
                             args.start_paused)

    with launch_interactive_simulation(runner):
        # Add a callback to record and print statistics
        runner.AddStepCallback(stats.record_tick)

        # Add a second callback that prints a message roughly every 500 calls
        runner.AddStepCallback(random_print)

        stats.start()

        runner.Start()


if __name__ == "__main__":
    main()