# Copyright 2019 Toyota Research Institute

"""
A module providing behaviour trees to work on agent-based simulation.
"""

import delphyne.blackboard
import delphyne.simulation

import py_trees.trees
from py_trees.trees import CONTINUOUS_TICK_TOCK
import time

class BehaviourTree(py_trees.trees.BehaviourTree):
    """
    A behaviour tree for agent-based simulations using Delphyne.
    """

    def __init__(self, *, root):
        super().__init__(root=root)
        self.runner = None

    def setup(self, realtime_rate, start_paused, time_step=0.001, logfile_name='', **kwargs):
        """
        Setup a Delphyne behaviour tree for agent based simulation.

        :param realtime_rate:
        :param start_paused:
        :param logfile_name:
        """
        # Bear in mind that this is a local builder. Once the AgentSimulation
        # is created, the builder will be destroyed, along with its blueprints
        builder = delphyne.simulation.AgentSimulationBuilder()

        super().setup(builder=builder, **kwargs)
        self.runner = delphyne.simulation.SimulationRunner(
            simulation=builder.build(),
            time_step=time_step,  # (secs)
            realtime_rate=realtime_rate,
            paused=start_paused,
            log=bool(logfile_name),
            logfile_name=logfile_name
        )
        delphyne.blackboard.state.set_simulation(
            self.runner.get_simulation()
        )

    def step(self, period):
        """
        Step simulation forward in time.

        :param period: in seconds.
        """
        self.runner.run_sync_for(period)

    def tick_tock(self,
                  period,
                  number_of_iterations=CONTINUOUS_TICK_TOCK,
                  pre_tick_handler=None,
                  post_tick_handler=None):
        tick_tocks = 0
        while (not self.interrupt_tick_tocking and (
                tick_tocks < number_of_iterations or
                number_of_iterations == CONTINUOUS_TICK_TOCK
        )):
            if not self.runner.is_simulation_paused():
                # Do not tick tree if simulation is paused.
                self.tick(pre_tick_handler, post_tick_handler)
                tick_tocks += 1
            try:
                # Always step simulation to unpause.
                self.step(period)
            except KeyboardInterrupt:
                break
        self.interrupt_tick_tocking = False
