import maliput.api as maliput
import random

import py_trees.blackboard

'''
The idea of this module is to have a commmon "interface" to set/get the
most common variables required for every (or almost) scenario.
We didn't inherit from the Blackboard class from pytrees to keep compatibility
with the existent behaviours that instantiates a blackboard.
If you desire to add a key to be usable in the setup phase of the behaviour
tree, keep on mind that the setup call is top-down.
'''

ROAD_GEOMETRY_KEY = 'road_geometry'
SIMULATION_KEY = 'simulation'
LANE_PROVIDER_KEY = 'lane_provider'

class LaneAndLocationProvider():

    road_geometries = {}
    seed = 0

    def __init__(self, distance_between_agents):
        self.distance_between_agents = distance_between_agents

    def load_lanes_from_road_geometry(self, road_geometry):
        road_id = road_geometry.id().string()
        lanes = (road_geometry.junction(i).segment(j).lane(k)
             for i in range(road_geometry.num_junctions())
             for j in range(road_geometry.junction(i).num_segments())
             for k in range(road_geometry.junction(i).segment(j).num_lanes()
                            )
                )
        lanes_dict = {}
        for lane in lanes:
            lane_id = lane.id().string()
            lanes_dict[lane_id] = []
            for lane_position in range(
                    int(lane.length() / self.distance_between_agents)):
                    lanes_dict[lane_id].append(
                        maliput.LanePosition(
                            s=lane_position * self.distance_between_agents,
                            r=0., h=0.))
            if len(lanes_dict[lane_id]) == 0:
                lanes_dict.pop(lane_id, None)
        self.road_geometries[road_id] = lanes_dict

    def random_lane(self, road_geometry):
        road_id = road_geometry.id().string()
        if road_id not in self.road_geometries:
            self.load_lanes_from_road_geometry(road_geometry)
        if len(self.road_geometries[road_id].values()) == 0:
            return None
        LaneAndLocationProvider.seed += 1
        return random.Random(LaneAndLocationProvider.seed).choice(
            list(self.road_geometries[road_id].keys()))

    def random_position(self, road_geometry, lane_id):
        road_id = road_geometry.id().string()
        if road_id not in self.road_geometries:
            self.load_lanes_from_road_geometry(road_geometry)
        lanes = self.road_geometries[road_id]
        if len(lanes.keys()) == 0 or \
            lane_id not in lanes:
            return None
        lane_position = lanes[lane_id].pop()
        if len(lanes[lane_id]) == 0:
            lanes.pop(lane_id, None)
        return lane_position

    def random_lane_and_position(self, road_geometry):
        lane_id = self.random_lane(road_geometry)
        return lane_id, self.random_position(lane_id, road_geometry)

    def amount_of_lanes(self, road_geometry):
        road_id = road_geometry.id().string()
        if road_id not in road_geometries:
            return 0
        return len(self.road_geometries[road_id].keys())


def get_road_geometry():
    blackboard = py_trees.blackboard.Blackboard()
    return blackboard.get(ROAD_GEOMETRY_KEY)

def set_road_geometry(road_geometry):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(ROAD_GEOMETRY_KEY, road_geometry, True)

def get_simulation():
    blackboard = py_trees.blackboard.Blackboard()
    return blackboard.get(SIMULATION_KEY)

def set_simulation(simulation):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(SIMULATION_KEY, simulation, True)

def initialize_agent_attributes(agent_name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(agent_name, {}, True)

def get_attribute_for_agent(agent_name, attribute):
    blackboard = py_trees.blackboard.Blackboard()
    attributes = blackboard.get(agent_name)
    if attribute in attributes:
        return attributes[attribute]
    else:
        return None
