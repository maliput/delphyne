#!/usr/bin/env python2.7

"""This is an example of running an automotive simulator and to control the
advance of the simulation by pressing specific keys on the keyboard.
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
from utils import (
    add_drake_resource_path,
    build_simple_car_simulator,
    launch_bridge,
    launch_visualizer
)


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


def run_simulation_loop(sim_runner, simulation_time_step):
    """Runs the keyboard-controlled simulation loop. Based on the key pressed
    the simulation will play/pause/step.
    """
    running = True
    paused = False

    keyboard = KeyboardHandler()
    print("\n*************************************************************\n"
          "* Instructions for running the demo:                        *\n"
          "* <p> will pause the simulation if unpaused and viceversa.  *\n"
          "* <s> will step the simulation once if paused.              *\n"
          "* <q> will stop the simulation and quit the demo.           *\n"
          "*************************************************************\n")
    print("Simulation is running")
    while running:
        if keyboard.key_hit():
            key = keyboard.get_character().lower()
            if key == 'p':
                paused = not paused
                if paused:
                    print("Simulation is paused")
                else:
                    print("Simulation is running")
            elif key == 'q':
                running = False
                print("Quitting simulation")
                break
            elif key == 's':
                if paused:
                    sim_runner.RunSimulationStep()
                    print("Simulation step of {0}s executed".
                          format(simulation_time_step))
                else:
                    print("Simulation step only supported in paused mode")
        elif not paused:
            sim_runner.RunSimulationStep()


def main():
    """Spawn an automotive simulator"""

    # Checks for env variables presence, quits the demo otherwise.
    try:
        add_drake_resource_path()
    except RuntimeError, error_msg:
        sys.stderr.write('ERROR: {}'.format(error_msg))
        sys.exit(1)

    launcher = Launcher()

    simulator = build_simple_car_simulator()

    try:
        launch_bridge(launcher)

        launch_visualizer(launcher, "layoutWithTeleop.config")

        simulation_time_step = 0.001

        runner = SimulatorRunner(simulator, simulation_time_step)

        run_simulation_loop(runner, simulation_time_step)

    finally:
        runner.Stop()
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


if __name__ == "__main__":
    main()
