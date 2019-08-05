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
