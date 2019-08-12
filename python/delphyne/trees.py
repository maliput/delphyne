import delphyne.simulation

import py_trees.trees

from py_trees.trees import CONTINUOUS_TICK_TOCK


class BehaviourTree(py_trees.trees.BehaviourTree):

    def __init__(self, *, root):
        super().__init__(root=root)
        self.runner = None

    def setup(self, realtime_rate, start_paused, logfile_name='', **kwargs):
        builder = delphyne.simulation.AgentSimulationBuilder()

        super().setup(builder=builder, **kwargs)

        self.runner = delphyne.simulation.SimulationRunner(
            simulation=builder.build(),
            time_step=0.001,  # (secs)
            realtime_rate=realtime_rate,
            paused=start_paused,
            log=bool(logfile_name),
            logfile_name=logfile_name
        )

    def step(self, period):
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
                self.tick(pre_tick_handler, post_tick_handler)
                tick_tocks += 1
            try:
                self.step(period)
            except KeyboardInterrupt:
                break
        self.interrupt_tick_tocking = False
