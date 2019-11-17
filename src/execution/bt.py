import py_trees
from skills.grasping import ActionGrasping
from skills.navigate import ActionNavigate
from skills.pddl_descriptions import get_action_description
from execution.condition_check import ConditionChecker_Blackboard, ConditionChecker_Predicate
from execution.custom_chooser import CustomChooser
from multiprocessing import Lock


class ExecutionSystem:
    def __init__(self, scene, robot, predicates, plan=None):
        self._scene = scene
        self._robot = robot
        self._predicates = predicates
        self.tree = None

        self._lock = Lock()

        # py_trees.logging.level = py_trees.logging.Level.DEBUG

        if plan is None:
            self.create_tree()
        else:
            self.create_tree_from_plan(plan)

    def create_tree(self):
        root = py_trees.composites.Selector("Selector")
        grasping = ActionGrasping(self._scene, self._robot, self._lock)
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
            action_arg_dict = {}
            if len(plan_item_list) > 2:
                for i in range(len(plan_item_list[2:])):
                    action_arg_dict[descr_action[1]["params"][i][0]] = plan_item_list[2+i]
            
            # Establish pre-conditions
            preconds = []
            for precond in descr_action[1]["preconds"]:
                precond_check = ConditionChecker_Predicate(self._predicates.call[precond[0]],
                                                            self.process_pred_args(precond[2], action_arg_dict),
                                                            lock=self._lock,
                                                            invert=precond[1])
                preconds.append(precond_check)
            preconds_node = py_trees.composites.Sequence(name="Precond root", children=preconds)

            # Establish effects
            effects = []
            for effect in descr_action[1]["effects"]:
                effect_check = ConditionChecker_Predicate(self._predicates.call[effect[0]],
                                                            self.process_pred_args(effect[2], action_arg_dict),
                                                            lock=self._lock,
                                                            invert=effect[1])
                effects.append(effect_check)
            effects_node = py_trees.composites.Sequence(name="Effect root", children=effects)

            # Build action run part of tree
            if action_name == "grasp":
                action_node = ActionGrasping(self._scene, self._robot, self._lock, target=("cube1", None, 0))
            elif action_name == "nav":
                action_node = ActionNavigate(self._scene, self._robot._model.uid, target_name="cube1")

            local_run_root = CustomChooser(name="Run root", children=[effects_node, action_node])
            local_can_run_root = CustomChooser(name="Can run root", children=[effects_node, preconds_node])

            local_root = py_trees.composites.Sequence(name="Root", children=[local_can_run_root, local_run_root])
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

    def process_pred_args(self, pred_args, arg_dict):
        for i in range(len(pred_args)):
            pred_args[i] = arg_dict[pred_args[i]]
            if pred_args[i] == "robot1":
                pred_args[i] = self._robot
        return tuple(pred_args)
