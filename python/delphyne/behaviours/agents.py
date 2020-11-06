# Copyright 2019 Toyota Research Institute

"""
A module providing basic agent behaviours.
"""

import delphyne.agents
from delphyne.blackboard.providers import resolve

import py_trees.behaviours
import py_trees.common


class SimpleCar(py_trees.behaviours.Success):
    """Introduce a simple car in the road."""

    def __init__(self, name=py_trees.common.Name.AUTO_GENERATED,
                 initial_pose=(0., 0., 0.), speed=1.):
        """
        :param name: a name for the car.
        :param initial_pose: initial car (SE2) pose in the world frame.
            That is, its (x, y, heading) coordinates, either as a tuple of
            floats or as a callable expression that takes a road geometry
            and returns a tuple of floats.
        :param speed: for the agent as measured in the world frame, in m/s.
        """
        super().__init__(name)
        self.initial_pose = initial_pose
        self.speed = speed

    def setup(self, *, builder):
        initial_x, initial_y, heading = resolve(
            self.initial_pose, builder.get_road_geometry()
        )
        builder.add_agent(
            delphyne.agents.SimpleCarBlueprint(
                name=self.name,
                x=initial_x,          # initial x-coordinate (m)
                y=initial_y,          # initial y-coordinate (m)
                heading=heading,      # heading (radians)
                speed=self.speed      # speed in the direction of travel (m/s)
            )
        )


class MobilCar(py_trees.behaviours.Success):
    """Introduce a MOBIL car in the road."""

    def __init__(self, name=py_trees.common.Name.AUTO_GENERATED,
                 initial_pose=(0., 0., 0.), speed=1.,
                 direction_of_travel=True):
        """
        :param name: a name for the car.
        :param initial_pose: initial car (SE2) pose in the world frame.
            That is, its (x, y, heading), either as a tuple of floats or
            as a callable expression that takes a road geometry and returns
            a tuple of floats.
        :param speed: speed of the car as measured in the world frame, in
            meters per second.
        :param direction_of_travel: with or against the lane s-direction.
        """
        super().__init__(name)
        self.initial_pose = initial_pose
        self.direction_of_travel = direction_of_travel
        self.speed = speed

    def setup(self, *, builder):
        road_geometry = builder.get_road_geometry()
        initial_x, initial_y, initial_heading = resolve(
            self.initial_pose, road_geometry
        )
        builder.add_agent(
            delphyne.agents.MobilCarBlueprint(
                name=self.name,                 # unique name
                # with or against the lane s-direction
                direction_of_travel=self.direction_of_travel,
                x=initial_x,                 # x-coordinate (m)
                y=initial_y,                 # y-coordinate (m)
                heading=initial_heading,     # heading (radians)
                speed=self.speed                  # the s-direction (m/s)
            )
        )


class RailCar(py_trees.behaviours.Success):
    """Introduce a rail car in the road."""

    def __init__(self, lane_id, direction_of_travel=True,
                 longitudinal_position=0., lateral_offset=0.,
                 speed=1., nominal_speed=20.,
                 name=py_trees.common.Name.AUTO_GENERATED):
        """
        :param lane_id: ID of the lane to drive on, either as a string or
            as a callable expression that takes a road geometry and returns
            a string.
        :param direction_of_travel: with or against the lane s-direction.
        :param longitudinal_position: initial s-coordinate of the car in the
            lane frame, either as a float or as a callable expression that
            takes a road geometry and a lane and returns a float.
        :param lateral_offset: initial r-coordinate in the lane frame.
        :param speed: speed of the car as measured in the world frame, in m/s.
        :param nominal_speed: nominal speed (as in, maximum most likely speed) of
            the agent as measured in the world frame, in m/s.
        :param name: a name for the car.
        """
        super().__init__(name)
        self.lane_id = lane_id
        self.direction_of_travel = direction_of_travel
        self.longitudinal_position = longitudinal_position
        self.lateral_offset = lateral_offset
        self.speed = speed
        self.nominal_speed = nominal_speed

    def setup(self, *, builder):
        road_geometry = builder.get_road_geometry()
        road_index = road_geometry.ById()
        lane_id = resolve(self.lane_id, road_geometry)
        lane = road_index.GetLane(lane_id)
        longitudinal_position = resolve(
            self.longitudinal_position, road_geometry, lane
        )

        builder.add_agent(
            delphyne.agents.RailCarBlueprint(
                name=self.name,                                   # unique name
                lane=lane,                                        # lane
                direction_of_travel=self.direction_of_travel,     # direction_of_travel
                longitudinal_position=longitudinal_position,      # lane s-coordinate (m)
                lateral_offset=self.lateral_offset,               # lane r-coordinate (m)
                speed=self.speed,                                 # initial speed in
                                                                  # s-direction (m/s)
                nominal_speed=self.nominal_speed                  # nominal_speed (m/s)
            )
        )


class TrajectoryAgent(py_trees.behaviours.Success):
    """Introduce a trajectory following car in the road."""

    def __init__(self, times, headings, waypoints,
                 name=py_trees.common.Name.AUTO_GENERATED):
        """
        :param times: time of each waypoint in the trajectory to occur,
            in seconds.
        :param headings: orientation about the world frame's Z axis of
            the car at each waypoint, in radians.
        :param waypoints: (x, y, z) position in the world frame of the
            car at each waypoint, in meters.
        :param name: a name for the car.
        """
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


class UnicycleCar(py_trees.behaviours.Success):
    """Introduce a unicycle car in the road."""

    def __init__(self, name=py_trees.common.Name.AUTO_GENERATED,
                 initial_pose=(0., 0., 0.), speed=1.):
        """
        :param name: a name for the car.
        :param initial_pose: initial car (SE2) pose in the world frame.
            That is, its (x, y, heading) coordinates, either as a tuple of
            floats or as a callable expression that takes a road geometry
            and returns a tuple of floats.
        :param speed: for the agent as measured in the world frame, in m/s.
        """
        super().__init__(name)
        self.initial_pose = initial_pose
        self.speed = speed

    def setup(self, *, builder):
        initial_x, initial_y, heading = resolve(
            self.initial_pose, builder.get_road_geometry()
        )
        builder.add_agent(
            delphyne.agents.UnicycleCarBlueprint(
                name=self.name,
                x=initial_x,          # initial x-coordinate (m)
                y=initial_y,          # initial y-coordinate (m)
                heading=heading,      # heading (radians)
                speed=self.speed      # speed in the direction of travel (m/s)
            )
        )
