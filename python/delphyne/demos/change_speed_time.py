#!/usr/bin/env python2.7
#
# Copyright 2018 Toyota Research Institute
#
"""
The demo to change railcar speed after a certain amount of time.
```
"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import os.path

import delphyne.maliput as maliput
import delphyne.simulation as simulation
import delphyne.utilities as utilities

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "Change Speed after Time",
        """
An example of a single railcar on a closed-loop maliput road that changes speed
after a fixed amount of time.
        """
    )

    return parser.parse_args()


class TimeMonitor(object):
    '''
    A class to monitor the time on every callback and perform an action after
    ten seconds have elapsed.
    '''
    def __init__(self, simulator, agent):
        self.simulator = simulator
        self.agent = agent
        self.changed_speed = False

    def check_tick(self):
        '''
        The callback called on every simulator iteration.  It checks
        to see if the elapsed simulator_time is greater than 10 seconds, and
        once it is, it changes the speed of the agent.
        '''
        if self.simulator.get_current_simulation_time() >= 10.0 \
           and not self.changed_speed:
            self.agent.set_speed(10.0)
            self.changed_speed = True


##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    simulator = simulation.AutomotiveSimulator()

    filename = "{0}/roads/circuit.yaml".format(
        utilities.get_delphyne_resource_root())

    if not os.path.isfile(filename):
        print("Required file {} not found."
              " Please, make sure to install the latest delphyne-gui."
              .format(os.path.abspath(filename)))
        quit()

    # The road geometry
    road_geometry = simulator.set_road_geometry(
        maliput.create_multilane_from_file(
            file_path=filename
        )
    )

    # Setup railcar
    railcar_speed = 4.0  # (m/s)
    railcar_s = 0.0      # (m)
    robot_id = 1
    lane_1 = road_geometry.junction(2).segment(0).lane(0)
    myagent = utilities.add_rail_car(
        simulator,
        name=str(robot_id),
        lane=lane_1,
        position=railcar_s,
        offset=0.0,
        speed=railcar_speed,
        road_geometry=road_geometry)

    runner = simulation.SimulatorRunner(
        simulator=simulator,
        time_step=0.015,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused,
        log=args.log,
        logfile_name=args.logfile_name
    )

    monitor = TimeMonitor(simulator, myagent)

    with utilities.launch_interactive_simulation(runner) as launcher:
        runner.add_step_callback(monitor.check_tick)

        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            print("Running simulation for {0} seconds.".format(
                args.duration))
            runner.run_async_for(args.duration, launcher.terminate)
