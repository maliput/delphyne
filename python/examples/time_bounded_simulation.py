#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
This example shows how to run a simulation for a fixed period of time (15
seconds by default). As an added bonus, the user can also specify the real-time
simulation rate (which by default is 1.0).

The following command depicts how to run a simulation for 30 sim seconds using
a real-time rate of 2x (so it should run in approximately 15 wall clock
seconds):

```
$ time_bounded_simulation.py --realtime_rate=2.0 --duration=30.0
```
"""

from __future__ import print_function

import argparse

from delphyne.bindings import SimulatorRunner
from delphyne.simulation_utils import (
    build_simple_car_simulator,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


def check_positive_float_or_zero(value):
    """Check that the passed argument is a positive float value"""
    float_value = float(value)
    if float_value < 0.0:
        raise argparse.ArgumentTypeError("%s is not a positive float value"
                                         % value)
    return float_value


def main():
    """Spawns a simulator_runner in paused mode"""

    # Read the initial real-time rate and simulation duration from command
    # line.
    parser = argparse.ArgumentParser(
        prog="time_bounded_simulation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-d", "--duration", default=15.0,
                        type=check_positive_float_or_zero,
                        help="The duration the simulation should run for")
    parser.add_argument("-r", "--realtime_rate", default=1.0,
                        type=check_positive_float_or_zero,
                        help="The real-time rate at which the simulation \
                             should start running")

    args = parser.parse_args()

    simulation_duration = args.duration

    realtime_rate = args.realtime_rate

    simulator = build_simple_car_simulator()

    runner = SimulatorRunner(simulator,
                             SIMULATION_TIME_STEP_SECS,
                             realtime_rate)

    with launch_interactive_simulation(runner) as launcher:

        print("Running simulation for {0} seconds @ {1}x real-time rate "
              .format(simulation_duration, realtime_rate))

        runner.RunAsyncFor(simulation_duration, launcher.terminate)


if __name__ == "__main__":
    main()
