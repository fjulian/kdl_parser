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
        N = len(plan)

        # Compute all actions' preconditions and effects
        all_preconds = []
        all_effects = []
        for k in range(N):
            plan_item = plan[k]
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
            preconds_node = py_trees.composites.Sequence(name="Preconds action {}".format(k+1), children=preconds)
            all_preconds.append(preconds_node)

            # Establish effects
            effects = []
            for effect in descr_action[1]["effects"]:
                effect_check = ConditionChecker_Predicate(self._predicates.call[effect[0]],
                                                            self.process_pred_args(effect[2], action_arg_dict),
                                                            lock=self._lock,
                                                            invert=effect[1])
                effects.append(effect_check)
            effects_node = py_trees.composites.Sequence(name="Effects action {}".format(k+1), children=effects)
            all_effects.append(effects_node)

        # Compute all subgoals
        all_goals = [None] * N
        for k in reversed(range(N)):
            # Establish goal
            goals = []
            # TODO set up goals

        # Set up actual tree
        next_lower_root = None
        for k in range(N):
            plan_item = plan[k]
            plan_item_list = plan_item.split(' ')
            action_name = plan_item_list[1]

            # Need run part
            effect_node = all_effects[k]
            precond_node = py_trees.composites.Sequence(children=all_preconds[k+1:])
            need_run_root = py_trees.composites.Sequence(name="Need run {}".format(k+1), children=[effect_node, precond_node])

            # Do run part
            if action_name == "grasp":
                action_node = ActionGrasping(self._scene, self._robot, self._lock, target=("cube1", None, 0))
            elif action_name == "nav":
                action_node = ActionNavigate(self._scene, self._robot._model.uid, target_name="cube1")
            do_run_root = py_trees.composites.Selector(name="Do run {}".format(k+1), children=[need_run_root, action_node])

            # Can run part
            can_run_root = py_trees.composites.Sequence(name="Can run {}".format(k+1), children=[all_goals[k], all_preconds[k]])

            # Precheck part
            if next_lower_root is not None:
                precheck_root = py_trees.composites.Selector(name="Precheck {}".format(k+1), children=[can_run_root, next_lower_root])
            else:
                precheck_root = can_run_root

            # Local root
            local_root = py_trees.composites.Sequence(name="Root {}".format(k+1), children=[precheck_root, do_run_root])
            next_lower_root = local_root
        
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
