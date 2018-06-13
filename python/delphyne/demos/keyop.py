#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
A keyop demo.
"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function
from select import select

import atexit
import sys
import termios
import time

import delphyne.simulation as simulation
import delphyne.utilities as utilities

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


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


def demo_callback(runner, launcher, keyboard_handler, time_step_seconds):
    """Callback function invoqued by the SimulatorRunner
    at every time step.
    """
    if keyboard_handler.key_hit():
        key = keyboard_handler.get_character().lower()
        if key == 'p':
            if runner.is_simulation_paused():
                runner.unpause_simulation()
                print("Simulation is now running.")
            else:
                runner.pause_simulation()
                print("Simulation is now paused.")
        elif key == 'q':
            runner.stop()
            print("Quitting simulation.")
            # This is needed to avoid a possible deadlock.
            # See SimulatorRunner class description.
            time.sleep(0.5)
            launcher.terminate()
        elif key == 's':
            if runner.is_simulation_paused():
                runner.request_simulation_step_execution(1)
                print("Simulation step of {0}s executed.".
                      format(time_step_seconds))
            else:
                print("Simulation step only supported in paused mode.")


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "Keyboard Teleoperation & Time Manipulation",
        """
This example shows how to use the keyboard events to control the
advance of a simulation. The simulation will open the usual simple
car in the center of the scene, which can be driven using the
keyboard on the GUI's teleop widget. In the console, we can toggle
(p) play/pause, (s) step and (q) quit the simulation.
        """
        )
    return parser.parse_args()

##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    simulator = simulation.AutomotiveSimulator()

    utilities.add_simple_car(
        simulator,
        name=str(0),
        position_x=0.0,
        position_y=0.0)

    simulation_time_step_secs = 0.001
    runner = simulation.SimulatorRunner(
        simulator,
        time_step=simulation_time_step_secs
    )

    keyboard = KeyboardHandler()

    runner.add_step_callback(
        lambda: demo_callback(runner, launcher, keyboard,
                              simulation_time_step_secs))

    print("\n"
          "************************************************************\n"
          "* Instructions for running the demo:                       *\n"
          "* <p> will pause the simulation if unpaused and viceversa. *\n"
          "* <s> will step the simulation once if paused.             *\n"
          "* <q> will stop the simulation and quit the demo.          *\n"
          "************************************************************\n")

    with utilities.launch_interactive_simulation(runner) as launcher:
        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            print("Running simulation for {0} seconds.".format(
                args.duration))
            runner.run_async_for(args.duration, launcher.terminate)
