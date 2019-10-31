import py_trees
from skills.grasping import ActionGrasping
from execution.condition_check import ConditionChecker


class ExecutionSystem:
    def __init__(self, scene, robot):
        self._scene = scene
        self._robot = robot
        self.tree = None

        self.create_tree()

    def create_tree(self):
        root = py_trees.composites.Selector("Selector")
        grasping = ActionGrasping(self._scene, self._robot)
        grasping_check = ConditionChecker("grasp_success")
        root.add_children([grasping_check, grasping])
        
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(timeout=15)
