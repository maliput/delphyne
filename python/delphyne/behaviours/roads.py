# Copyright 2019 Toyota Research Institute

"""
A module providing road composites.
"""

import delphyne.roads
import delphyne.blackboard

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
            # Setup a road geometry only the first time.
            self.road_geometry = builder.set_road_geometry(
                delphyne.roads.create_dragway(
                    name=self.name,
                    num_lanes=self.num_lanes,
                    length=self.length,
                    lane_width=self.lane_width,
                    shoulder_width=self.shoulder_width,
                    maximum_height=self.maximum_height
                )
            )
            delphyne.blackboard.state.set_road_geometry(self.road_geometry)


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
            # Setup a road geometry only the first time.
            self.road_geometry = builder.set_road_geometry(
                delphyne.roads.create_multilane_from_file(
                    self.file_path)
                )
            delphyne.blackboard.state.set_road_geometry(self.road_geometry)


class Malidrive(Road):
    """
    A maliput malidrive road.
    """

    def __init__(self, file_path, features=delphyne.roads.ObjFeatures(),
                 name=py_trees.common.Name.AUTO_GENERATED):
        super().__init__(name)
        self.file_path = file_path
        self.features = features
        self.road_network = None

    def setup(self, *, builder):
        if self.road_network is None:
            # Setup a road network only the first time.
            self.road_network = builder.set_road_network(
                delphyne.roads.create_malidrive_from_file(
                    name=self.name,
                    file_path=self.file_path), self.features)
            self.road_geometry = self.road_network.road_geometry()
            delphyne.blackboard.state.set_road_geometry(self.road_geometry)
