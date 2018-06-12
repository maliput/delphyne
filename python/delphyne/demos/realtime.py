#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#
# TODO(daniel.stonier) this demo would be easier if there were
# agents on a road moving from the start (currently have to teleop it).
"""
Manipulating the rate of time at startup and in runtime.
"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import os
import sys

import delphyne.simulation as simulation  # pylint: disable=no-name-in-module
import delphyne.utilities as utilities

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


class RealtimeRateChanger(object):

    """Simple class that hooks to the simulation callback and dynamically
    changes the real-time rate"""

    def __init__(self, runner, initial_steps):
        self._runner = runner
        self._steps = initial_steps

    def tick(self):
        """Process simulation step"""
        if self._steps == 0:
            last_round_realtime_rate = self._runner.get_stats(
            ).get_current_realtime_rate()
            rate = self._runner.get_realtime_rate() + 0.2
            if rate >= 1.6:
                rate = 0.6
            self._steps = int(rate * 3000)
            self._runner.set_realtime_rate(rate)
            print("Running at real-time rate {0} for {1} steps."
                  " Last real-time measure was {2}"
                  .format(rate, self._steps, last_round_realtime_rate))
        else:
            self._steps -= 1


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "Realtime Rate Changer",
        """
This example shows how the real-time simulation rate can be set both when the
simulator runner is created and while the simulation is running.

To pass an initial real-time rate use the `--realtime_rate` flag, like:

$ {0} --realtime_rate=2.0

If none is specified the default will be set to `1.0` (i.e. run the simulation
in real-time).

Once the scripts starts running it will cycle between a real-time rate of `0.6`
to `1.6` to depict how dynamic real-time rate impacts on the simulation.
        """.format(os.path.basename(sys.argv[0])))
    return parser.parse_args()

##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    # Read the initial real-time rate from command line. Default to 1.0 if none
    # specified.
    args = parse_arguments()

    # Since this is the first time the simulator runs we compensate for the
    # startup time by running it 4 times longer than the dynamically changing
    # loop.
    initial_steps = int(args.realtime_rate * 12000)

    simulator = simulation.AutomotiveSimulator()
    utilities.add_simple_car(
        simulator,
        name=str(0),
        position_x=0.0,
        position_y=0.0)

    runner = simulation.SimulatorRunner(
        simulator=simulator,
        time_step=0.001,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused
    )

    rate_changer = RealtimeRateChanger(runner, initial_steps)

    runner.add_step_callback(rate_changer.tick)

    print("Running at real-time rate {0} for {1} steps"
          .format(runner.get_realtime_rate(), initial_steps))

    with utilities.launch_interactive_simulation(runner) as launcher:
        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            runner.run_async_for(args.duration, launcher.terminate)
