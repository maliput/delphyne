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
            # When `lane.length()`` is less than `step_size` a lane position at the
            # start of the lane is expected to be created so as to to guarantee at
            # least one lane position per lane.
            step_count = max(int(lane.length() / step_size), 1)
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
