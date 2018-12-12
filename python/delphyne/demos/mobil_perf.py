#!/usr/bin/env python2.7
#
# Copyright 2018 Toyota Research Institute
#
"""
The performance benchmark demo.
"""
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import math

import delphyne.maliput as maliput
import delphyne.simulation as simulation
import delphyne.utilities as utilities

from . import helpers

##############################################################################
# Supporting Classes & Methods
##############################################################################


def benchmark(setup_fn):
    """
    Helper decorator to register benchmark simulation
    setup functions.
    """
    if not hasattr(benchmark, 'register'):
        benchmark.register = {}
    benchmark.register[setup_fn.__name__] = setup_fn
    return setup_fn


@benchmark
def curved_lanes(args):
    """
    Sets up a simulation with `args.num_cars` MOBIL cars on a few
    lanes of a curved (arc) road.
    """
    builder = simulation.AgentSimulationBuilder()

    # Loads Multilane road.
    road = builder.set_road_geometry(
        maliput.create_multilane_from_file(
            "{0}/roads/curved_lanes.yaml".format(
                utilities.get_delphyne_resource_root()
            )
        )
    )

    # Adds the N MOBIL cars to the multilane.
    R0 = 320.  # m
    for i in range(args.num_cars):
        R = R0 - 4. * (i % 3)  # m
        # For a 6m distance between cars.
        theta = (12./R0) * (i / 3)  # rads
        utilities.add_mobil_car(
            builder, name="mobil" + str(i),
            scene_x=R * math.sin(theta),  # m
            scene_y=R0 - R * math.cos(theta),  # m
            heading=theta,  # rads
            speed=1.0,  # m/s
        )

    # Adds the N*T rail cars to the multilane.
    road_segment = road.junction(0).segment(0)
    num_traffic = int(args.traffic_density * args.num_cars)
    for i in range(num_traffic):
        utilities.add_rail_car(
            builder, name="rail " + str(i),
            lane=road_segment.lane(i % 3),
            position=12. * (i / 3) + 6.,  # m
            offset=0.,  # m
            speed=1.0  # m/s
        )

    return builder.build()


@benchmark
def straight_lanes(args):
    """
    Sets up a simulation with `args.num_cars` MOBIL cars on a few
    lanes of a straight road.
    """

    builder = simulation.AgentSimulationBuilder()

    # Loads Multilane road.
    road = builder.set_road_geometry(
        maliput.create_multilane_from_file(
            file_path="{0}/roads/straight_lanes.yaml".format(
                utilities.get_delphyne_resource_root()
            )
        )
    )

    # Adds the N MOBIL cars to the multilane.
    for i in range(args.num_cars):
        utilities.add_mobil_car(
            builder, name="mobil" + str(i),
            scene_x=12. * (i / 3),  # m
            scene_y=4. * (i % 3),  # m
            heading=0.0,  # rads
            speed=1.0,  # m/s
        )

    # Adds the N*T rail cars to the multilane.
    road_segment = road.junction(0).segment(0)
    num_traffic = int(args.traffic_density * args.num_cars)
    for i in range(num_traffic):
        utilities.add_rail_car(
            builder, name="rail " + str(i),
            lane=road_segment.lane(i % 3),
            position=12. * (i / 3) + 6.,  # m
            offset=0.,  # m
            speed=1.0   # m/s
        )

    return builder.build()


@benchmark
def dragway(args):
    """
    Sets up a simulation with `args.num_cars` MOBIL cars on a dragway
    road with four (4) lanes.
    """
    builder = simulation.AgentSimulationBuilder()

    road = builder.set_road_geometry(
        maliput.create_dragway(
            name="dragway",
            num_lanes=4,
            length=100.0,  # m
            lane_width=3.7,  # m
            shoulder_width=3.0,  # m
            maximum_height=5.0  # m
        )
    )

    # Adds the N MOBIL cars to the dragway.
    for i in range(args.num_cars):
        utilities.add_mobil_car(
            builder, name="mobil" + str(i),
            scene_x=12.0 * (i / 4),  # m
            scene_y=-5.5 + 3.7 * (i % 4),  # m
            heading=0.0,  # rads
            speed=1.0,  # m/s
        )

    # Adds the N*T rail cars to the multilane.
    road_segment = road.junction(0).segment(0)
    num_traffic = int(args.traffic_density * args.num_cars)
    for i in range(num_traffic):
        utilities.add_rail_car(
            builder, name="rail " + str(i),
            lane=road_segment.lane(i % 4),
            position=12. * (i / 4) + 6.,  # m
            offset=0.,  # m
            speed=1.0   # m/s
        )

    return builder.build()


def parse_arguments():
    "Argument passing and demo documentation."
    parser = helpers.create_argument_parser(
        "MOBIL Performance Check",
        "\nCPU hungry MOBIL cars on common roads.\n",
        default_duration=5.0)
    available_benchmarks = benchmark.register.keys()
    parser.add_argument(
        "benchmark", choices=available_benchmarks,
        help="Benchmark to be run."
    )
    parser.add_argument(
        "-t", "--traffic-density", default=0, type=float,
        help=("The number of rail cars per "
              "MOBIL car on scene (default: 0).")
    )
    parser.add_argument(
        "-n", "--num-cars", default=20, type=int,
        help="The number of MOBIL cars on scene (default: 20)."
    )
    return parser.parse_args()


##############################################################################
# Main
##############################################################################


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    benchmark_simulation = benchmark.register[args.benchmark](args)

    runner = simulation.SimulationRunner(
        benchmark_simulation,
        time_step=0.01,  # (secs)
        realtime_rate=args.realtime_rate,
        paused=args.paused,
        log=args.log,
        logfile_name=args.logfile_name
    )

    if args.duration < 0:
        # run indefinitely
        runner.start()
    else:
        # run for a finite amount of time
        print("Running simulation for {0} seconds.".format(
            args.duration))
        runner.run_sync_for(args.duration)
    # stop simulation if it's necessary
    if runner.is_interactive_loop_running():
        runner.stop()
    # print simulation stats
    print("Simulation ended. I'm happy, you should be too.")
    utilities.print_simulation_stats(runner)
