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


def get_road_geometry():
    blackboard = py_trees.blackboard.Blackboard()
    return blackboard.get(ROAD_GEOMETRY_KEY)

def set_road_geometry(road_geometry):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(ROAD_GEOMETRY_KEY, road_geometry, True)
