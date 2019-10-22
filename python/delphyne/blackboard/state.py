# Copyright 2019 Toyota Research Institute

"""
A module to maintain scenario state in the blackboard.
"""

import py_trees.blackboard


ROAD_GEOMETRY_KEY = 'road_geometry'
SIMULATION_KEY = 'simulation'


def get_road_geometry():
    blackboard = py_trees.blackboard.BlackboardClient(read={ROAD_GEOMETRY_KEY})
    return blackboard.get(ROAD_GEOMETRY_KEY)


def set_road_geometry(road_geometry):
    blackboard = py_trees.blackboard.BlackboardClient(write={ROAD_GEOMETRY_KEY})
    blackboard.set(ROAD_GEOMETRY_KEY, road_geometry, True)


def get_simulation():
    blackboard = py_trees.blackboard.BlackboardClient(read={SIMULATION_KEY})
    return blackboard.get(SIMULATION_KEY)


def set_simulation(simulation):
    blackboard = py_trees.blackboard.BlackboardClient(write={SIMULATION_KEY})
    blackboard.set(SIMULATION_KEY, simulation, True)


def initialize_agent_attributes(agent_name):
    blackboard = py_trees.blackboard.BlackboardClient(write={agent_name})
    blackboard.set(agent_name, {}, True)


def get_attribute_for_agent(agent_name, attribute_name):
    blackboard = py_trees.blackboard.BlackboardClient(read={agent_name})
    attributes = blackboard.get(agent_name)
    return attributes.get(attribute_name, None)


def set_attribute_for_agent(agent_name, attribute_name, attribute_value):
    blackboard = py_trees.blackboard.BlackboardClient(read={agent_name})
    attributes = blackboard.get(agent_name)
    attributes[attribute_name] = attribute_value
