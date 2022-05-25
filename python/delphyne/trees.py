# BSD 3-Clause License
#
# Copyright (c) 2022, Woven Planet. All rights reserved.
# Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
A module providing behaviour trees to work on agent-based simulation.
"""

import delphyne.blackboard
import delphyne.simulation

import py_trees.trees
from py_trees.trees import CONTINUOUS_TICK_TOCK


class BehaviourTree(py_trees.trees.BehaviourTree):
    """
    A behaviour tree for agent-based simulations using Delphyne.
    """

    def __init__(self, *, root):
        super().__init__(root=root)
        self.runner = None

    def setup(self, realtime_rate, start_paused, time_step=0.01,
              log=False, logfile_name='', **kwargs):
        """
        Setup a Delphyne behaviour tree for agent based simulation.

        :param realtime_rate:
        :param start_paused:
        :param time_step:
        :param log:
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
            log=log,
            logfile_name=logfile_name
        )

        for node in self.root.iterate():
            if not hasattr(node, 'late_setup'):
                continue
            node.late_setup(simulation=self.runner.get_simulation())

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

        assert(self.runner.get_timestep() <= period),                                     \
            "Sim runner time step ({}) must be less than or equal to tree time step ({})" \
            .format(self.runner.get_timestep(), period)

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
