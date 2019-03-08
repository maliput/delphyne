#!/usr/bin/env python3
#
# Copyright 2017 Toyota Research Institute
#

"""This is a minimal example of starting an automotive simulation using a
python binding to the C++ `SimulationRunner` class.

Note that this is not a configurable demo, it will just create a sample
simulation with a prius car that can be driven around and road tests
a couple of python scriptlets (callbacks) in the simulator's tick-tock.

Check the other examples in this directory for more advanced uses.
"""
##############################################################################
# Imports
##############################################################################

import os.path

import random
import time

import delphyne.maliput as maliput
import delphyne.simulation as simulation
import delphyne.utilities as utilities

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


class SimulationStats(object):
    """This is a simple class to keep statistics of the simulation, just
    averaging the time it takes to execute a simulation step from the outside
    world. Every 1000 measures, the values are printed to stdout.
    """

    def __init__(self):
        """Just init the stats"""
        self.reset()
        self._current_start_time = None

    def reset(self):
        """Clear all values"""
        self._time_sum = 0.0
        self._samples_count = 0
        self._current_start_time = None

    def print_stats(self):
        """Print the stats"""
        print(
            "Average simulation step takes {delta}ms"
            .format(delta=(self._time_sum / self._samples_count) * 1000))

    def start(self):
        """Record the time when we start measuring"""
        self._current_start_time = time.time()

    def record_tick(self):
        """A simulation tick happened. Record it.
        Every 1000 ticks print the stats and reset
        """
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
    if random.randint(1, 500) == 1:
        print("One in five hundred")


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "Scriptlets",
        """
This is a minimal example demonstrating the inclusion of a python
callback (scriptlet) to be triggered at each tick of the simulation.
        """
    )
    return parser.parse_args()


class TimeMonitor(object):
    '''
    A class to monitor the time on every callback and perform an action after
    ten seconds have elapsed.
    '''
    def __init__(self, sim, agent):
        self.simulation = sim
        self.agent = agent
        self.changed_speed = False

    def check_tick(self):
        '''
        The callback called on every simulator iteration.  It checks
        to see if the elapsed simulator_time is greater than 10 seconds, and
        once it is, it changes the speed of the agent.
        '''
        if self.simulation.get_current_time() >= 10.0 \
           and not self.changed_speed:
            self.agent.set_speed(20.0)
            self.changed_speed = True


##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    builder = simulation.AgentSimulationBuilder()

    filename = utilities.get_delphyne_resource('roads/circuit.yaml')

    if not os.path.isfile(filename):
        print("Required file {} not found."
              " Please, make sure to install the latest delphyne-gui."
              .format(os.path.abspath(filename)))
        quit()

    # The road geometry
    road_geometry = builder.set_road_geometry(
        maliput.create_multilane_from_file(
            file_path=filename
        )
    )

    simple_car_name = "simple0"
    utilities.add_simple_car(
        builder,
        name=simple_car_name,
        position_x=0.0,
        position_y=0.0
    )

    # Setup railcar
    railcar_speed = 4.0  # (m/s)
    railcar_s = 0.0      # (m)
    rail_car_name = "rail0"
    lane_1 = road_geometry.junction(2).segment(0).lane(0)
    rail_car_blueprint = utilities.add_rail_car(
        builder,
        name=rail_car_name,
        lane=lane_1,
        position=railcar_s,
        offset=0.0,
        speed=railcar_speed
    )

    runner = simulation.SimulationRunner(
        simulation=builder.build(),
        time_step=0.001,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused,
        log=args.log,
        logfile_name=args.logfile_name
    )

    running_simulation = runner.get_simulation()
    rail_car = rail_car_blueprint.get_mutable_agent(running_simulation)
    monitor = TimeMonitor(running_simulation, rail_car)

    stats = SimulationStats()
    with utilities.launch_interactive_simulation(runner) as launcher:
        # Add a callback to record and print statistics
        runner.add_step_callback(stats.record_tick)

        # Add a second callback that prints a message roughly every 500 calls
        runner.add_step_callback(random_print)

        # Add a third callback to check on the time elapsed and change speed
        runner.add_step_callback(monitor.check_tick)

        stats.start()

        if args.duration < 0:
            # run indefinitely
            runner.start()
        else:
            # run for a finite time
            print("Running simulation for {0} seconds.".format(
                args.duration))
            runner.run_async_for(args.duration, launcher.terminate)
