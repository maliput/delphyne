#!/usr/bin/python2
#
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

import argparse
import lcm
import os
import select
import sys
import time
from launcher import Launcher


def wait_for_lcm_message_on_channel(channel):
    """Wait for a single message to arrive on the specified LCM channel.
    """
    m = lcm.LCM()

    def receive(channel, data):
        _ = (channel, data)
        raise StopIteration()

    sub = m.subscribe(channel, receive)
    start_time = time.time()
    try:
        while True:
            if time.time() - start_time > 10.:
                raise RuntimeError(
                    "Timeout waiting for channel %s" % channel)
            rlist, _, _ = select.select([m], [], [], 0.1)
            if m in rlist:
                m.handle()
    except StopIteration:
        pass
    finally:
        m.unsubscribe(sub)

def get_from_env_or_fail(var):
    value = os.environ.get(var)
    if value is None:
        print("%s is not in the environment, did you remember to source setup.bash?" % (var))
        sys.exit(1)

    # Since it is an environment variable, the very end may have a colon; strip it here
    if value[-1] == ':':
        value = value[:-1]

    return value

def main():
    """Launches drake's automotive_demo along with the bridge and the
    ignition-visualizer.
    It can spawn optionally the drake-visualizer and the lcm tools.
    Terminates all the processes if one of them is killed.
    """
    launcher = Launcher()
    parser = argparse.ArgumentParser()
    demo_arguments = {
        "simple": ["--num_simple_car=2"],
        "trajectory": ["--num_trajectory_car=1"],
        "dragway":  ["--num_dragway_lanes=3", "--num_trajectory_car=12"],
    }
    # Number of cars on each demo, passed as arguments to the bridge
    # this approach is temporal, and will be removed as soon as the
    # dynamic creation of lcm-to-ign repeaters is ready
    num_cars = {"simple": "2", "trajectory": "1", "dragway": "12"}

    # Optional arguments
    parser.add_argument("--demo", default="simple", dest="demo_name",
                        action="store", choices=demo_arguments.keys(),
                        help="the specific demo to launch")

    parser.add_argument("--no-drake-visualizer", action="store_false",
                        default=True, dest="drake_visualizer",
                        help="don't launch drake-visualizer")

    args, tail = parser.parse_known_args()

    drake_src_dir = get_from_env_or_fail('DRAKE_SRC_DIR')
    delphyne_ws_dir = get_from_env_or_fail('DELPHYNE_WS_DIR')

    # Build up the binary path
    drake_bazel_bin_path = os.path.join(drake_src_dir, 'bazel-bin')

    # Delphyne binaries; these are found through the standard PATH, so are relative
    lcm_ign_bridge = "duplex-ign-lcm-bridge"
    ign_visualizer = "visualizer"

    # The drake-visualizer, drake-lcm-spy, and lcm-logger are installed when
    # installing drake, so use PATH
    drake_visualizer = "drake-visualizer"
    drake_lcm_spy = "drake-lcm-spy"
    lcm_logger = "lcm-logger"

    # The automotive_demo and steering_command_driver binaries from drake.
    # These aren't installed with a drake install, so we must run them from the
    # drake src directory.
    demo_path = os.path.join(drake_bazel_bin_path, "drake", "automotive", "automotive_demo")
    steering_command_driver_path = os.path.join(drake_bazel_bin_path, "drake", "automotive", "steering_command_driver")

    try:
        launcher.launch([lcm_ign_bridge, num_cars[args.demo_name]])

        # TODO: replace this delay with a
        # feedback from the ignition visualizer
        time.sleep(1)

        if args.demo_name == "simple":
            # Load custom layout with two TeleopWidgets
            teleop_config = os.path.join(delphyne_ws_dir, "install", "share", "delphyne", "layoutWithTeleop.config")
            launcher.launch([ign_visualizer, teleop_config])
        else:
            launcher.launch([ign_visualizer])


        # TODO: Remove this once backend changes are in
        time.sleep(1)

        if args.drake_visualizer:
            if args.demo_name == "simple":
                # Launch two instances of the drake steering_command app
                launcher.launch([steering_command_driver_path, "--lcm_tag=DRIVING_COMMAND_0"])
                launcher.launch([steering_command_driver_path, "--lcm_tag=DRIVING_COMMAND_1"])
            launcher.launch([drake_lcm_spy])
            launcher.launch([lcm_logger])
            launcher.launch([drake_visualizer])
            # wait for the drake-visualizer to be up
            wait_for_lcm_message_on_channel("DRAKE_VIEWER_STATUS")
        else:
            # TODO: once we have the backend with a service for startup, this
            # can go away.
            time.sleep(1)

        launcher.launch([demo_path] + demo_arguments[args.demo_name], cwd=drake_src_dir)

        launcher.wait(float("Inf"))

    finally:
        launcher.kill()

if __name__ == "__main__":
    main()
