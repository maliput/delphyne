#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
This example shows how the real-time simulation rate can be set both when the
simulator runner is created and while the simulation is running.

To pass an initial real-time rate use the `--realtime_rate` flag, like:

```
$ realtime_rate_changer.py --realtime_rate=2.0
```

If none is specified the default will be set to `1.0` (i.e. run the simulation
in real-time).

Once the scripts starts running it will cycle between a real-time rate of `0.6`
to `1.6` to depict how dynamic real-time rate impacts on the simulation.
"""

from __future__ import print_function

import argparse

from delphyne import SimulatorRunner
from simulation_utils import (
    build_simple_car_simulator,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP = 0.001


def check_positive_float(value):
    """Check that the passed argument is a positive float value"""
    float_value = float(value)
    if float_value <= 0.0:
        raise argparse.ArgumentTypeError("%s is not a positive float value"
                                         % value)
    return float_value


class RealtimeRateChanger(object):
    """Simple class that hooks to the simulation callback and dynamically
    changes the real-time rate"""

    def __init__(self, runner, initial_steps):
        self._runner = runner
        self._steps = initial_steps

    def tick(self):
        """Process simulation step"""
        if self._steps == 0:
            rate = self._runner.GetRealtimeRate() + 0.2
            if rate >= 1.6:
                rate = 0.6
            self._steps = int(rate * 3000)
            self._runner.SetRealtimeRate(rate)
            print("Running at real-time rate {0} for {1} steps"
                  .format(rate, self._steps))
        else:
            self._steps -= 1


def main():
    """Spawns a simulator_runner in paused mode"""

    # Read the initial real-time rate from command line. Default to 1.0 if none
    # specified.
    parser = argparse.ArgumentParser(
        prog="realtime_rate_changer",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-r", "--realtime_rate", default=1.0,
                        type=check_positive_float,
                        help="""The real-time rate at which the simulation \
                             should start running""")
    args = parser.parse_args()

    initial_realtime_rate = args.realtime_rate

    # Since this is the first time the simulator runs we compensate for the
    # startup time by running it 4 times longer than the dynamically changing
    # loop.
    initial_steps = int(initial_realtime_rate * 12000)

    simulator = build_simple_car_simulator()

    runner = SimulatorRunner(simulator,
                             SIMULATION_TIME_STEP,
                             initial_realtime_rate)

    rate_changer = RealtimeRateChanger(runner, initial_steps)

    runner.AddStepCallback(rate_changer.tick)

    with launch_interactive_simulation(runner):
        print("Running at real-time rate {0} for {1} steps"
              .format(runner.GetRealtimeRate(), initial_steps))
        runner.Start()


if __name__ == "__main__":
    main()
