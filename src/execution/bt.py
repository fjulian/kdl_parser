import py_trees
from skills.grasping import ActionGrasping


class ExecutionSystem:
    def __init__(self, scene, robot):
        self._scene = scene
        self._robot = robot
        self.tree = None

        self.create_tree()

    def create_tree(self):
        root = py_trees.composites.Selector("Selector")
        grasping = ActionGrasping(self._scene, self._robot)
        root.add_children([grasping])
        
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(timeout=15)
