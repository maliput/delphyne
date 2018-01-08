#!/usr/bin/env python

"""Launches drake's automotive_demo along with the bridge and the
ignition-visualizer.
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
import os
import sys
from launcher import Launcher
from utils import get_from_env_or_fail


def wait_for_lcm_message_on_channel(channel):
    """Wait for a single message to arrive on the specified LCM channel.
    """
    lcm_connection = lcm.LCM()

    def receive(channel, data):
        """LCM channel handler"""
        _ = (channel, data)
        raise StopIteration()

    sub = lcm_connection.subscribe(channel, receive)
    start_time = time.time()
    try:
        while True:
            if time.time() - start_time > 10.:
                raise RuntimeError(
                    "Timeout waiting for channel %s" % channel)
            rlist, _, _ = select.select([lcm_connection], [], [], 0.1)
            if lcm_connection in rlist:
                lcm_connection.handle()
    except StopIteration:
        pass
    finally:
        lcm_connection.unsubscribe(sub)


def main():
    """Launches drake's automotive_demo along with the bridge and the
    ignition-visualizer.
    It can spawn optionally the drake-visualizer and the lcm tools.
    Terminates all the processes if one of them is killed.

    Important: Many parts of this file are commented out as we are currently
    migrating the demos and not all features are supported. We are leaving
    these commented-out lines as a reference for the future.
    """
    launcher = Launcher()
    parser = argparse.ArgumentParser()
    demo_arguments = {
        "simple": ["--num_simple_car=1"]
    }

    # Number of cars on each demo, passed as arguments to the bridge
    # this approach is temporal, and will be removed as soon as the
    # dynamic creation of lcm-to-ign repeaters is ready
    num_cars = {"simple": "1", "trajectory": "1", "dragway": "12"}

    # Optional arguments
    parser.add_argument("--demo", default="simple", dest="demo_name",
                        action="store", choices=demo_arguments.keys(),
                        help="the specific demo to launch")

    args = parser.parse_args()

    delphyne_ws_dir = get_from_env_or_fail('DELPHYNE_WS_DIR')

    # Delphyne binaries; these are found through the standard PATH, so
    # they are relative
    lcm_ign_bridge = "duplex-ign-lcm-bridge"
    ign_visualizer = "visualizer"
    automotive_demo = "automotive-demo"

    try:
        launcher.launch([lcm_ign_bridge, num_cars[args.demo_name]])

        if args.demo_name == "simple":
            # Load custom layout with two TeleopWidgets
            teleop_config = os.path.join(
                delphyne_ws_dir,
                "install",
                "share",
                "delphyne",
                "layoutWithTeleop.config")
            launcher.launch([ign_visualizer, teleop_config])
        else:
            launcher.launch([ign_visualizer])

        launcher.launch([automotive_demo])
        launcher.wait(float("Inf"))

    finally:
        launcher.kill()


if __name__ == "__main__":
    main()
