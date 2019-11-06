import py_trees
from skills.grasping import ActionGrasping
from skills.pddl_descriptions import get_action_description
from execution.condition_check import ConditionChecker_Blackboard, ConditionChecker_Predicate


class ExecutionSystem:
    def __init__(self, scene, robot, predicates):
        self._scene = scene
        self._robot = robot
        self._predicates = predicates
        self.tree = None

        py_trees.logging.level = py_trees.logging.Level.DEBUG

        self.create_tree()

    def create_tree(self):
        root = py_trees.composites.Selector("Selector")
        grasping = ActionGrasping(self._scene, self._robot)
        grasping_check = ConditionChecker_Blackboard("grasp_success")
        root.add_children([grasping_check, grasping])
        
        self.tree = py_trees.trees.BehaviourTree(root)
        self.show_tree()
        self.tree.setup(timeout=15)

    def create_tree_from_plan(self, plan):
        next_lower_root = None
        for plan_item in reversed(plan):
            plan_item_list = plan_item.split(' ')
            action_name = plan_item_list[1]
            descr_action = get_action_description(action_name)

            local_root = py_trees.composites.Sequence()
            
            # Establish pre-conditions
            preconds = []
            for precond in descr_action[1]["preconds"]:
                precond_check = ConditionChecker_Predicate(self._predicates.call[precond[0]], self.process_pred_args(precond[2]))
                if precond[1]:
                    # Negated is set to true
                    precond_check = py_trees.decorators.Inverter(child=precond_check)
                preconds.append(precond_check)
            local_preconds_root = py_trees.composites.Sequence(children=preconds)

            # Establish effects
            effects = []
            for effect in descr_action[1]["effects"]:
                effect_check = ConditionChecker_Predicate(self._predicates.call[effect[0]], self.process_pred_args(effect[2]))
                if effect[1]:
                    # Negated is set to true
                    effect_check = py_trees.decorators.Inverter(child=effect_check)
                effects.append(effect_check)
            effects_node = py_trees.composites.Sequence(children=effects)

            # Build action run part of tree
            action_node = ActionGrasping(self._scene, self._robot)


            effect_root = py_trees.composites.Sequence()


    def show_tree(self):
        print("="*20)
        print("Behavior tree:")
        print("-"*20)
        py_trees.display.print_ascii_tree(self.tree.root)
        print("="*20)

    def process_pred_args(self, pred_args):
        for i in range(pred_args):
            if pred_args[i] == "robot1":
                pred_args[i] = self._robot
        return tuple(pred_args)
