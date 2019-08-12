import delphyne.roads

import py_trees.composites
import py_trees.common

class Road(py_trees.composites.Sequence):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.road_geometry = None

    def iterate(self, *args, **kwargs):
        # Ensure roads are setup once before their children.
        yield self
        yield from super().iterate(*args, **kwargs)


class Dragway(Road):

    def __init__(self, num_lanes, length, lane_width, shoulder_width,
                 maximum_height, name=py_trees.common.Name.AUTO_GENERATED):
        super().__init__(name)
        self.num_lanes = num_lanes
        self.length = length
        self.lane_width = lane_width
        self.shoulder_width = shoulder_width
        self.maximum_height = maximum_height

    def setup(self, *, builder):
        if self.road_geometry is None:
            self.road_geometry = builder.set_road_geometry(
                delphyne.maliput.create_dragway(
                    name=self.name,
                    num_lanes=self.num_lanes,
                    length=self.length,
                    lane_width=self.lane_width,
                    shoulder_width=self.shoulder_width,
                    maximum_height=self.maximum_height
                )
            )
