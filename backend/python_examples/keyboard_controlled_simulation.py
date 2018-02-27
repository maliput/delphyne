#!/usr/bin/env python2.7
"""
This example shows how to use the keyboard events to control the advance of
a simulation. The simulation will open the usual simple car in the center of
the scene, which can be driven using the keyboard on the GUI's teleop widget.
However, by switching to the console, we can `play`/`pause`/`step`/`quit` the
simulation.

```
$ cd <delphyne_ws>/install/bin
$ ./keyboard_controlled_simulation.py
```

 The supported keys for the demo:

<`p`> will pause the simulation if running and vice-versa.

<`s`> will step the simulation once if paused.

<`q`> will stop the simulation and quit the demo.
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
from select import select

import atexit
import sys
import termios
import time

from launcher import Launcher
from simulation_runner_py import SimulatorRunner
from delphyne_utils import (
    add_drake_resource_path,
    build_simple_car_simulator,
    launch_visualizer
)

SIMULATION_TIME_STEP = 0.001


class KeyboardHandler(object):
    """A keyboard-interrupt poller. Allows users to read a keyboard
    input with a non-locking behavior making use of the select function,
    available on most *nix systems.

    This class is based on the work done by Frank Deng, available on GitHub
    as part of a set of python tools released under the MIT licence:
    https://github.com/frank-deng/experimental-works/blob/master/kbhit.py .
    """

    def __init__(self, input_stream=None):
        if input_stream:
            self.input_stream = input_stream
        else:
            self.input_stream = sys.stdin
        # Save current terminal settings
        self.file_descriptor = self.input_stream.fileno()
        self.new_terminal = termios.tcgetattr(self.file_descriptor)
        self.old_terminal = termios.tcgetattr(self.file_descriptor)
        # New terminal setting unbuffered
        self.new_terminal[3] = (self.new_terminal[3] &
                                ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.file_descriptor,
                          termios.TCSAFLUSH, self.new_terminal)
        # Support normal-terminal reset at exit
        atexit.register(self.set_normal_term)

    def set_normal_term(self):
        """Resets to default terminal settings."""
        termios.tcsetattr(self.file_descriptor,
                          termios.TCSAFLUSH, self.old_terminal)

    def get_character(self):
        """Reads a character from the keyboard."""
        char = self.input_stream.read(1)
        if char == '\x00' or ord(char) >= 0xA1:
            return char + self.input_stream.read(1)
        return char

    def key_hit(self):
        """Returns True if a keyboard key has been pressed, False otherwise."""
        key_hit, _, _ = select([self.input_stream], [], [], 0)
        return key_hit != []


def demo_callback(runner, launcher, keyboard_handler):
    """Callback function invoqued by the SimulatorRunner
    every SIMULATION_TIME_STEP seconds.
    """
    if keyboard_handler.key_hit():
        key = keyboard_handler.get_character().lower()
        if key == 'p':
            if runner.IsPaused():
                runner.Unpause()
                print("Simulation is now running.")
            else:
                runner.Pause()
                print("Simulation is now paused.")
        elif key == 'q':
            runner.Stop()
            print("Quitting simulation.")
            # This is needed to avoid a possible deadlock.
            # See SimulatorRunner class description.
            time.sleep(0.5)
            launcher.terminate()
        elif key == 's':
            if runner.IsPaused():
                runner.RequestStep(SIMULATION_TIME_STEP)
                print("Simulation step of {0}s executed.".
                      format(SIMULATION_TIME_STEP))
            else:
                print("Simulation step only supported in paused mode.")


def main():
    """Runs the demo."""
    try:
        # Checks for env variables presence, quits the demo otherwise.
        add_drake_resource_path()

        launcher = Launcher()
        simulator = build_simple_car_simulator()

        runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP)

        keyboard = KeyboardHandler()

        launch_visualizer(launcher, "layoutWithTeleop.config")

        runner.AddStepCallback(
            lambda: demo_callback(runner, launcher, keyboard))
        runner.Start()

        print(
            "\n*************************************************************\n"
            "* Instructions for running the demo:                        *\n"
            "* <p> will pause the simulation if unpaused and viceversa.  *\n"
            "* <s> will step the simulation once if paused.              *\n"
            "* <q> will stop the simulation and quit the demo.           *\n"
            "*************************************************************\n"
            "Simulation is now running.\n")

        launcher.wait(float("Inf"))

    except RuntimeError, error_msg:
        sys.stderr.write('ERROR: {}'.format(error_msg))
        sys.exit(1)


if __name__ == "__main__":
    main()
