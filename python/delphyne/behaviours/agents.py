import delphyne.agents
import delphyne.blackboard.blackboard_helper as bb_helper

import py_trees.behaviours
import py_trees.common

def resolve(expression, *args, **kwargs):
    if callable(expression):
        expression = expression(*args, **kwargs)
    return expression

class SimpleCar(py_trees.behaviours.Success):

    def __init__(self, name=py_trees.common.Name.AUTO_GENERATED,
                 initial_x=0., initial_y=0., heading=0., speed=1.):
        super().__init__(name)
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.heading = heading
        self.speed = speed

    def setup(self, *, builder):
        builder.add_agent(
            delphyne.agents.SimpleCarBlueprint(
                name=self.name,
                x=self.initial_x,  # initial x-coordinate (m)
                y=self.initial_y,  # initial y-coordinate (m)
                heading=self.heading,   # heading (radians)
                speed=self.speed      # speed in the direction of travel (m/s)
            )
        )


class MobilCar(py_trees.behaviours.Success):

    '''
    MobilCar agent. You can give a function to lane_pos_generator parameter
    that returns a pair (Lane, LanePosition)
    '''
    def __init__(self, name=py_trees.common.Name.AUTO_GENERATED,
                 initial_pose=(0., 0., 0.), # x, y and initial heading coord.
                 speed=1., direction_of_travel=True):
        super().__init__(name)
        self.initial_pose = initial_pose
        self.direction_of_travel = direction_of_travel
        self.speed = speed

    def setup(self, *, builder):
        road_geometry = bb_helper.get_road_geometry()
        self.initial_pose = resolve(self.initial_pose, road_geometry)
        builder.add_agent(
            delphyne.agents.MobilCarBlueprint(
                name=self.name,                 # unique name
                # with or against the lane s-direction
                direction_of_travel=self.direction_of_travel,
                x=self.initial_pose[0],                 # x-coordinate (m)
                y=self.initial_pose[1],                 # y-coordinate (m)
                heading=self.initial_pose[2],     # heading (radians)
                speed=self.speed                  # the s-direction (m/s)
            )
        )


class RailCar(py_trees.behaviours.Success):

    def __init__(self, lane_id, direction_of_travel=True,
                 longitudinal_position=0., lateral_offset=0.,
                 speed=1., nominal_speed=20.,
                 name=py_trees.common.Name.AUTO_GENERATED):
        super().__init__(name)
        self.lane_id = lane_id
        self.direction_of_travel = direction_of_travel
        self.longitudinal_position = longitudinal_position
        self.lateral_offset = lateral_offset
        self.speed = speed
        self.nominal_speed = nominal_speed
        self.agent = None
        bb_helper.initialize_agent_attributes(self.name)

    def initialise(self):
        if self.agent is None:
            self.agent = bb_helper.get_simulation().get_agent_by_name(self.name)

    def setup(self, *, builder):
        road_geometry = builder.get_road_geometry()
        road_index = road_geometry.ById()
        self.lane_id = resolve(self.lane_id, road_geometry)
        self.longitudinal_position = resolve(self.longitudinal_position,
            road_geometry, self.lane_id)
        lane = road_index.GetLane(self.lane_id)
        builder.add_agent(
            delphyne.agents.RailCarBlueprint(
                name=self.name,                                   # unique name
                lane=lane,                                        # lane
                direction_of_travel=self.direction_of_travel,     # direction_of_travel
                longitudinal_position=self.longitudinal_position, # lane s-coordinate (m)
                lateral_offset=self.lateral_offset,               # lane r-coordinate (m)
                speed=self.speed,                                 # initial speed in
                                                                  # s-direction (m/s)
                nominal_speed=self.nominal_speed                  # nominal_speed (m/s)
            )
        )

    def update(self):
        speed = bb_helper.get_attribute_for_agent(self.name, "speed")
        if self.agent is not None and speed is not None and speed != self.speed:
            self.speed = self.agent.set_speed(speed)
        return py_trees.common.Status.SUCCESS



class TrajectoryAgent(py_trees.behaviours.Success):

    def __init__(self, times, headings, waypoints,
                 name=py_trees.common.Name.AUTO_GENERATED):
        super().__init__(name)
        self.times = times
        self.headings = headings
        self.waypoints = waypoints

    def setup(self, *, builder):
        builder.add_agent(
            delphyne.agents.TrajectoryAgentBlueprint(
                self.name,
                self.times,       # timings (sec)
                self.headings,    # list of headings (radians)
                self.waypoints   # list of x-y-z-tuples (m, m, m)
            )
        )

