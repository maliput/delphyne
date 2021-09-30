# Copyright 2019 Toyota Research Institute

"""
A module that holds data providers for scenario configuration.
"""

import random

import maliput.api as maliput


def resolve(expression, *args, **kwargs):
    """Resolve if a callable expression."""
    if callable(expression):
        expression = expression(*args, **kwargs)
    return expression


class LaneLocationProvider():
    """A random lane locations' provider."""

    def __init__(self, distance_between_agents, seed=1):
        """
        :param distance_between_agents: leeway distance between agents,
            in meters.
        :param seed: random generator seed integer.
        """
        self.distance_between_agents = distance_between_agents
        self.random_generator = random.Random(seed)
        self.road_locations = {}

    def _load_from(self, road_geometry):
        """
        Load all available lane locations from a ``road_geometry``.
        """
        lanes = (
            road_geometry.junction(i).segment(j).lane(k)
            for i in range(road_geometry.num_junctions())
            for j in range(road_geometry.junction(i).num_segments())
            for k in range(road_geometry.junction(i).segment(j).num_lanes())
        )
        lane_locations = {}
        for lane in lanes:
            lane_id = lane.id().string()
            step_size = self.distance_between_agents
            step_count = int(lane.length() / step_size)
            step_count = 1 if step_count == 0 else step_count
            lane_locations[lane_id] = [
                maliput.LanePosition(
                    s=i * step_size, r=0., h=0.
                ) for i in range(step_count)
            ]
        # TODO(hidmic): add Python bindings to retrieve true ID
        road_id = hash(road_geometry)
        self.road_locations[road_id] = lane_locations

    def random_lane(self, road_geometry):
        """
        Get a random lane in a road geometry.

        :param road_geometry: a `maliput.api.RoadGeometry` instance.
        :return: a random lane ID.
        """
        # TODO(hidmic): add Python bindings to retrieve true ID
        road_id = hash(road_geometry)
        if road_id not in self.road_locations:
            self._load_from(road_geometry)
        if not self.road_locations[road_id]:
            return None
        return self.random_generator.choice(
            list(self.road_locations[road_id].keys())
        )

    def random_lane_position(self, road_geometry, lane_id):
        """
        Get a random lane position in a road geometry.

        :param road_geometry: a `maliput.api.RoadGeometry` instance.
        :param lane_id: ID of a lane in the given ``road_geometry``.
        :return: a random `maliput.api.LanePosition`.
        """
        # TODO(hidmic): add Python bindings to retrieve true ID
        road_id = hash(road_geometry)
        if road_id not in self.road_locations:
            self._load_from(road_geometry)
        lane_locations = self.road_locations[road_id]
        if lane_id not in lane_locations:
            return None
        self.random_generator.shuffle(lane_locations[lane_id])
        return lane_locations[lane_id].pop()
