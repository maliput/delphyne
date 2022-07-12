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
A module providing road composites.
"""

import delphyne.roads
import delphyne.maliput as maliput

import py_trees.composites
import py_trees.common


class Road(py_trees.composites.Sequence):
    """
    A sequence composite for behaviours that take place in a road (i.e. world).

    Subclasses are expected to take care of road setup in simulation.
    As children may require road access during construction, roads go
    through a two stage setup: once before their children, once afterwards.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.road_geometry = None

    def iterate(self, *args, **kwargs):
        # Ensure roads are setup once before their children.
        yield self
        yield from super().iterate(*args, **kwargs)


class Dragway(Road):
    """
    A maliput dragway road.
    """

    def __init__(self, num_lanes, length, lane_width, shoulder_width,
                 maximum_height, name=py_trees.common.Name.AUTO_GENERATED):
        """
        :param num_lanes: number of dragway lanes.
        :param length: length of the dragway lanes, in meters.
        :param lane_width: width of the dragway lanes, in meters.
        :param shoulder_width: should width for dragway lanes, in meters.
        :param maximum_height: h upper bound for dragway lanes, in meters.
        """
        super().__init__(name)
        self.num_lanes = num_lanes
        self.length = length
        self.lane_width = lane_width
        self.shoulder_width = shoulder_width
        self.maximum_height = maximum_height

    def setup(self, *, builder):
        if self.road_geometry is None:
            # Setup a road network only the first time.
            self.road_geometry = builder.set_road_network(
                delphyne.roads.create_dragway(
                    name=self.name,
                    num_lanes=self.num_lanes,
                    length=self.length,
                    lane_width=self.lane_width,
                    shoulder_width=self.shoulder_width,
                    maximum_height=self.maximum_height
                )
            )


class Multilane(Road):
    """
    A maliput multilane road.
    """

    def __init__(self, file_path, name=py_trees.common.Name.AUTO_GENERATED):
        """
        :param file_path: path to the YAML description file of a maliput
            multilane road geometry.
        :param name: for the road geometry to be loaded.
        """
        super().__init__(name)
        self.file_path = file_path

    def setup(self, *, builder):
        if self.road_geometry is None:
            # Setup a road network only the first time.
            self.road_geometry = builder.set_road_network(
                delphyne.roads.create_multilane_from_file(
                    self.file_path)
                )


class Malidrive(Road):
    """
    A maliput malidrive road.
    """

    def __init__(self, name=py_trees.common.Name.AUTO_GENERATED,
                 file_path="", rule_registry_file_path="", road_rulebook_file_path="",
                 traffic_light_book_path="", phase_ring_path="", intersection_book_path="",
                 linear_tolerance=1e-3, angular_tolerance=1e-3,
                 features=delphyne.roads.ObjFeatures()):
        super().__init__(name)
        self.road_network = None
        self.name = name
        self.file_path = file_path
        self.rule_registry_file_path = rule_registry_file_path
        self.road_rulebook_file_path = road_rulebook_file_path
        self.traffic_light_book_path = traffic_light_book_path
        self.phase_ring_path = phase_ring_path
        self.intersection_book_path = intersection_book_path
        self.linear_tolerance = linear_tolerance
        self.angular_tolerance = angular_tolerance
        self.features = features

    def setup(self, *, builder):
        if self.road_network is None:
            # Setup a road network only the first time.
            self.road_network = builder.set_road_network(
                delphyne.roads.create_malidrive_road_network_from_xodr(
                    name=self.name,
                    file_path=self.file_path,
                    rule_registry_file_path=self.rule_registry_file_path,
                    road_rulebook_file_path=self.road_rulebook_file_path,
                    traffic_light_book_path=self.traffic_light_book_path,
                    phase_ring_path=self.phase_ring_path,
                    intersection_book_path=self.intersection_book_path,
                    linear_tolerance=self.linear_tolerance,
                    angular_tolerance=self.angular_tolerance), self.features)
            self.road_geometry = self.road_network.road_geometry()


class OnRamp(Road):
    """
    A maliput onramp road.
    """

    def setup(self, *, builder):
        if self.road_geometry is None:
            self.road_geometry = builder.set_road_network(
                delphyne.roads.create_on_ramp()
            )
