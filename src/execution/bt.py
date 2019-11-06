import py_trees
from skills.grasping import ActionGrasping
from skills.pddl_descriptions import get_action_description
from execution.condition_check import ConditionChecker_Blackboard, ConditionChecker_Predicate


class ExecutionSystem:
    def __init__(self, scene, robot, predicates, plan=None):
        self._scene = scene
        self._robot = robot
        self._predicates = predicates
        self.tree = None

        py_trees.logging.level = py_trees.logging.Level.DEBUG

        if plan is None:
            self.create_tree()
        else:
            self.create_tree_from_plan(plan)

    def create_tree(self):
        root = py_trees.composites.Selector("Selector")
        grasping = ActionGrasping(self._scene, self._robot)
        grasping_check = ConditionChecker_Blackboard("grasp_success")
        root.add_children([grasping_check, grasping])
        
        self.tree = py_trees.trees.BehaviourTree(root)
        self.show_tree()
        self.tree.setup(timeout=15)

    def create_tree_from_plan(self, plan):
        # next_lower_root = None
        for plan_item in reversed(plan):
            plan_item_list = plan_item.split(' ')
            action_name = plan_item_list[1]
            descr_action = get_action_description(action_name)
            
            # Establish pre-conditions
            preconds = []
            for precond in descr_action[1]["preconds"]:
                precond_check = ConditionChecker_Predicate(self._predicates.call[precond[0]], self.process_pred_args(precond[2]), invert=precond[1])
                preconds.append(precond_check)
            preconds_node = py_trees.composites.Sequence(children=preconds)

            # Establish effects
            effects = []
            for effect in descr_action[1]["effects"]:
                effect_check = ConditionChecker_Predicate(self._predicates.call[effect[0]], self.process_pred_args(effect[2]), invert=effect[1])
                effects.append(effect_check)
            effects_node = py_trees.composites.Sequence(children=effects)

            # Build action run part of tree
            action_node = ActionGrasping(self._scene, self._robot, target=("cube1", None, None))

            local_run_root = py_trees.composites.Selector(children=[effects_node, action_node])
            local_can_run_root = py_trees.composites.Selector(children=[effects_node, preconds_node])

            local_root = py_trees.composites.Sequence(children=[local_can_run_root, local_run_root])
        root = local_root
        self.tree = py_trees.trees.BehaviourTree(root)
        self.show_tree()
        self.tree.setup(timeout=15)


    def show_tree(self):
        print("="*20)
        print("Behavior tree:")
        print("-"*20)
        py_trees.display.print_ascii_tree(self.tree.root)
        print("="*20)

    def process_pred_args(self, pred_args):
        for i in range(len(pred_args)):
            if pred_args[i] == "rob":
                pred_args[i] = self._robot
        return tuple(pred_args)
