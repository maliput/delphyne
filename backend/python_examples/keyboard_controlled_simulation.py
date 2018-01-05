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
import os
import sys
import termios
import time

from launcher import Launcher
from pydrake.common import AddResourceSearchPath
from simulation_runner_py import (
    AutomotiveSimulator,
    SimpleCarState,
    SimulatorRunner
)


def get_from_env_or_fail(var):
    """Retrieves an env variable for a given name, fails if not found."""
    value = os.environ.get(var)
    if value is None:
        print("%s is not in the environment,"
              "did you remember to source setup.bash?" % (var))
        sys.exit(1)

    # Since it is an environment variable, the very end may have a colon;
    # strip it here
    return value.rstrip(':')


class KBHit(object):
    """A standard keyboard-interrupt poller. Allows users to read a keyboard
    input with a non-locking behavior making use of the select function,
    available on most *nix systems.

    This class is based on the work done by Simon D. Levy, published on his
    personal website http://home.wlu.edu/~levys/software/kbhit.py
    under the GNU GPL v3 licence.
    """

    def __init__(self):
        # Save the terminal settings
        self.file_descriptor = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.file_descriptor)
        self.old_term = termios.tcgetattr(self.file_descriptor)
        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.file_descriptor,
                          termios.TCSAFLUSH, self.new_term)
        # Support normal-terminal reset at exit
        atexit.register(self.set_normal_term)

    def set_normal_term(self):
        """Resets to normal terminal."""
        termios.tcsetattr(self.file_descriptor,
                          termios.TCSAFLUSH, self.old_term)

    @staticmethod
    def getch():
        """Returns a keyboard character after kbhit() has been called."""
        return sys.stdin.read(1)

    @staticmethod
    def kbhit():
        """Returns True if keyboard character was hit, False otherwise."""
        key_hit, _, _ = select([sys.stdin], [], [], 0)
        return key_hit != []


def create_and_init_automotive_sim():
    """Instantiates and initializes the automotive simulator."""
    simulator = AutomotiveSimulator()
    state = SimpleCarState()
    state.y = 0.0
    simulator.AddPriusSimpleCar("0", "DRIVING_COMMAND_0", state)

    return simulator


def main():
    """Spawn an automotive simulator"""
    launcher = Launcher()

    delphyne_ws_dir = get_from_env_or_fail('DELPHYNE_WS_DIR')
    lcm_ign_bridge = "duplex-ign-lcm-bridge"
    ign_visualizer = "visualizer"

    drake_install_path = get_from_env_or_fail('DRAKE_INSTALL_PATH')
    AddResourceSearchPath(os.path.join(drake_install_path, "share", "drake"))

    simulator = create_and_init_automotive_sim()
    try:
        launcher.launch([lcm_ign_bridge, "1"])
        teleop_config = os.path.join(delphyne_ws_dir,
                                     "install",
                                     "share",
                                     "delphyne",
                                     "layoutWithTeleop.config")
        launcher.launch([ign_visualizer, teleop_config])

        runner = SimulatorRunner(simulator, 0.001)

        running = True
        paused = False

        keyboard = KBHit()
        print("Simulation is running")
        while running:
            if keyboard.kbhit():
                key = keyboard.getch()
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
                        runner.RunSimulationStep()
                        print("Simulation step executed")
            elif not paused:
                runner.RunSimulationStep()

    finally:
        runner.Stop()
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


if __name__ == "__main__":
    main()
