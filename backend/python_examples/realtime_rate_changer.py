#!/usr/bin/env python2.7

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

# Copyright 2017 Open Source Robotics Foundation
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

from __future__ import print_function

import argparse
import sys
import time

from launcher import Launcher
from simulation_runner_py import SimulatorRunner
from delphyne_utils import (
    build_simple_car_simulator,
    launch_visualizer
)


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

    launcher = Launcher()

    simulator = build_simple_car_simulator()

    runner = SimulatorRunner(simulator, 0.001, initial_realtime_rate)

    rate_changer = RealtimeRateChanger(runner, initial_steps)

    runner.AddStepCallback(rate_changer.tick)

    try:
        launch_visualizer(launcher, "layoutWithTeleop.config")
        runner.Start()
        print("Running at real-time rate {0} for {1} steps"
              .format(runner.GetRealtimeRate(), initial_steps))
        launcher.wait(float("Inf"))
    finally:
        runner.Stop()
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


if __name__ == "__main__":
    main()
