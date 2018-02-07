#!/usr/bin/env python2.7

"""This is a minimal example of starting an automotive simulation using a
python binding to the C++ SimulatorRunner class.

Note that this is not a configurable demo, it will just create a sample
simulation with a prius car that can be driven around.

As we add more python bindings to the C++ classes we will start doing more
interesting scripts.
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

import os
import random
import sys
import time

from launcher import Launcher
from simulation_runner_py import SimulatorRunner


def get_from_env_or_fail(var):
    """Retrieves an env variable for a given name, fails if not found."""
    value = os.environ.get(var)
    if value is None:
        print("%s is not in the environment,"
              "did you remember to source setup.bash?" % (var))
        sys.exit(1)

    # Since it is an environment variable, the very end may have a colon;
    # strip it here
    if value[-1] == ':':
        value = value[:-1]

    return value


class SimulationStats(object):
    """This is a simple class to keep statistics of the simulation, just
    averaging the time it takes to execute a simulation step from the outside
    world. Every 1000 measures, the values are printed to stdout"""
    def __init__(self):
        self.reset()

    def reset(self):
        self._time_sum = 0.0
        self._samples_count = 0
        self._current_start_time = None

    def print_stats(self):
        print(
            "Average simulation step takes {delta}ms"
            .format(delta=(self._time_sum / self._samples_count) * 1000))

    def start(self):
        self._current_start_time = time.time()

    def record_tick(self):
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
    if (random.randint(1, 500) == 1):
        print("One in five hundred")


def main():
    """Spawn an automotive simulator making use of the python bindings"""
    stats = SimulationStats()
    launcher = Launcher()

    delphyne_ws_dir = get_from_env_or_fail('DELPHYNE_WS_DIR')
    lcm_ign_bridge = "duplex-ign-lcm-bridge"
    ign_visualizer = "visualizer"

    try:
        launcher.launch([lcm_ign_bridge, "1"])
        layout_key = "--layout="
        teleop_config = layout_key + os.path.join(delphyne_ws_dir,
                                                  "install",
                                                  "share",
                                                  "delphyne",
                                                  "layoutWithTeleop.config")
        launcher.launch([ign_visualizer, teleop_config])

        # TODO(basicNew) We need this to make sure the visualizer is up and
        # running before we send the initial message to load the robot.
        # This will go away once we have ported the automotive simulator to
        # the backend.
        time.sleep(1.0)

        runner = SimulatorRunner()
        # Add a callback to record and print statistics
        runner.AddStepCallback(stats.record_tick)
        # Add a second callback that prints a message roughly every 500 calls
        runner.AddStepCallback(random_print)

        stats.start()
        runner.Start()

        launcher.wait(float("Inf"))

    finally:
        runner.Stop()
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


if __name__ == "__main__":
    main()
