#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

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

from __future__ import print_function
from select import select

import atexit
import sys
import termios
import time

from delphyne.bindings import SimulatorRunner
from delphyne.simulation_utils import (
    build_simple_car_simulator,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


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
    every SIMULATION_TIME_STEP_SECS seconds.
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
                runner.RequestMultiStep(1)
                print("One simulation step executed.")
            else:
                print("Simulation step only supported in paused mode.")


def main():
    """Runs the demo."""
    simulator = build_simple_car_simulator()

    runner = SimulatorRunner(simulator, SIMULATION_TIME_STEP_SECS)

    keyboard = KeyboardHandler()

    runner.AddStepCallback(
        lambda: demo_callback(runner, launcher, keyboard))

    with launch_interactive_simulation(runner) as launcher:
        print("\n"
              "************************************************************\n"
              "* Instructions for running the demo:                       *\n"
              "* <p> will pause the simulation if unpaused and viceversa. *\n"
              "* <s> will step the simulation once if paused.             *\n"
              "* <q> will stop the simulation and quit the demo.          *\n"
              "************************************************************\n")

        runner.Start()


if __name__ == "__main__":
    main()
