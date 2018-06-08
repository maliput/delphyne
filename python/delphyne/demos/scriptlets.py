#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""This is a minimal example of starting an automotive simulation using a
python binding to the C++ `SimulatorRunner` class.

Note that this is not a configurable demo, it will just create a sample
simulation with a prius car that can be driven around and road tests
a couple of python scriptlets (callbacks) in the simulator's tick-tock.

Check the other examples in this directory for more advanced uses.
"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import random
import time

import delphyne.simulation as simulation  # pylint: disable=no-name-in-module
import delphyne.utilities as utilities

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


class SimulationStats(object):
    """This is a simple class to keep statistics of the simulation, just
    averaging the time it takes to execute a simulation step from the outside
    world. Every 1000 measures, the values are printed to stdout.
    """

    def __init__(self):
        """Just init the stats"""
        self.reset()
        self._current_start_time = None

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


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "Scriptlets",
        """
This is a minimal example demonstrating the inclusion of a python
callback (scriptlet) to be triggered at each tick of the simulation.
        """
    )
    return parser.parse_args()

##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    stats = SimulationStats()

    simulator = helpers.build_simple_car_simulator()

    runner = simulation.SimulatorRunner(
        simulator=simulator,
        time_step=0.001,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused
    )

    with utilities.launch_interactive_simulation(runner) as launcher:
        # Add a callback to record and print statistics
        runner.add_step_callback(stats.record_tick)

        # Add a second callback that prints a message roughly every 500 calls
        runner.add_step_callback(random_print)

        stats.start()

        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            print("Running simulation for {0} seconds.".format(
                args.duration))
            runner.run_async_for(args.duration, launcher.terminate)
