import pickle
from copy import deepcopy
from os import path, makedirs
from highlevel_planning.pddl_interface.pddl_file_if import PDDLFileInterface
from highlevel_planning.pddl_interface import planner_interface


def check_path_exists(path_to_check):
    if not path.isdir(path_to_check):
        makedirs(path_to_check)


class KnowledgeBase(object):
    def __init__(self, knowledge_dir, domain_name=""):
        # Folder book keeping
        domain_dir = path.join(knowledge_dir, "main")
        check_path_exists(domain_dir)
        problem_dir = domain_dir
        temp_domain_dir = path.join(knowledge_dir, "explore")
        check_path_exists(temp_domain_dir)
        temp_problem_dir = temp_domain_dir

        # Domain definition
        self._domain_name = domain_name
        self._predicates = dict()
        self.actions = dict()
        self.types = dict()

        self._domain_file = path.join(domain_dir, "_domain.pkl")
        self.load_domain()

        # Problem definition
        self.objects = dict()
        self.initial_predicates = list()
        # self.goals = list()
        # self.goals = [("in-hand", True, ("cube1", "robot1"))]
        self.goals = [("on", True, ("cupboard", "cube1"))]
        # self.goals = [("inside", True, ("container1", "cube1"))]

        # PDDL file interfaces
        self.pddl_if = PDDLFileInterface(domain_dir, problem_dir, domain_name)
        self.pddl_if_temp = PDDLFileInterface(
            temp_domain_dir, temp_problem_dir, domain_name
        )

        # Value lookups (e.g. for positions)
        self.lookup_table = dict()

        # Temporary variables (e.g. for exploration)
        self._temp_goals = list()
        self._temp_objects = dict()

    # ----- Loading and saving pickle with domain and problem ---------------------------

    def load_domain(self):
        print("Trying to load domain file...")
        if path.exists(self._domain_file):
            with open(self._domain_file, "rb") as f:
                load_obj = pickle.load(f)
            self._domain_name = load_obj[0]
            self._predicates = load_obj[1]
            self.actions = load_obj[2]
            self.types = load_obj[3]
            print("Trying to load domain file... DONE")
        else:
            print("Trying to load domain file... NOT FOUND --> starting from scratch")

    def save_domain(self):
        save_obj = (self._domain_name, self._predicates, self.actions, self.types)
        with open(self._domain_file, "wb") as f:
            pickle.dump(save_obj, f)
        print("Saved domain file")

    # ----- Adding to the domain description ------------------------------------

    def add_action(self, action_name, action_definition, overwrite=False):
        if not overwrite and action_name in self.actions:
            print(
                "Action "
                + action_name
                + " already exists and no overwrite was requested. Ignoring request."
            )
        else:
            assert isinstance(action_name, str)
            self.actions[action_name] = action_definition

    def add_predicate(self, predicate_name, predicate_definition, overwrite=False):
        if not overwrite and predicate_name in self._predicates:
            print(
                "Predicate "
                + predicate_name
                + " already exists and no overwrite was requested. Ignoring request."
            )
        else:
            assert isinstance(predicate_name, str)
            self._predicates[predicate_name] = predicate_definition

    def add_type(self, new_type, parent_type=None):
        assert isinstance(new_type, str)
        if new_type in self.types:
            if parent_type not in self.types[new_type]:
                self.types[new_type].append(parent_type)
        else:
            self.types[new_type] = [parent_type]

    # ----- Adding to the problem description ----------------------------------

    def add_object(self, object_name, object_type, object_value=None):
        assert object_type in self.types
        if object_name in self.objects:
            if object_type not in self.objects[object_name]:
                self.objects[object_name].append(object_type)
        else:
            self.objects[object_name] = [object_type]
        if object_value is not None:
            self.lookup_table[object_name] = object_value

    def add_objects(self, object_dict):
        for obj, obj_type in object_dict.items():
            self.add_object(obj, obj_type)

    def add_inital_predicates(self, pred_list):
        self.initial_predicates += pred_list

        # Remove duplicates
        self.initial_predicates = list(dict.fromkeys(self.initial_predicates))

    def add_goal(self, goal_list):
        self.goals += goal_list

        # Remove duplicates
        self.goals = list(dict.fromkeys(self.goals))

    def add_planning_problem(self, planning_problem):
        self.add_objects(planning_problem.objects)
        self.add_inital_predicates(planning_problem.initial_predicates)
        self.add_goal(planning_problem.goals)

    def clear_planning_problem(self):
        self.objects.clear()
        del self.initial_predicates[:]
        del self.goals[:]

    # ----- Solving ------------------------------------------------------------

    def solve(self):
        self.save_domain()
        self.pddl_if.write_domain_pddl(self.actions, self._predicates, self.types)
        self.pddl_if.write_problem_pddl(
            self.objects, self.initial_predicates, self.goals
        )
        return planner_interface.pddl_planner(
            self.pddl_if._domain_file_pddl, self.pddl_if._problem_file_pddl
        )

    # ----- Utilities ----------------------------------------------------------

    def test_goals(self, predicates):
        for goal in self.goals:
            if not predicates.call[goal[0]](*goal[2]):
                return False
        return True

    def check_predicates(self, predicates):
        """
        If predicates need to be initialized when the system is launched, this can be done here.
        
        Args:
            predicates ([type]): [description]
            robot ([type]): [description]
        """

        if predicates.empty_hand("robot1"):
            self.initial_predicates.append(("empty-hand", "robot1"))
        self.initial_predicates.append(("in-reach", "origin", "robot1"))

        # Check any predicates in relation with the goal
        for goal in self.goals:
            if predicates.call[goal[0]](*goal[2]):
                pred_tuple = (goal[0],) + goal[2]
                self.initial_predicates.append(pred_tuple)

    def populate_objects(self, scene):
        # TODO maybe move this into a separate dummy perception module
        for obj in scene.objects:
            self.objects[obj] = ["item"]

    def _type_is_position(self, type_to_check):
        if type_to_check == "position":
            return True
        elif type_to_check is None or self.types[type_to_check] == [None]:
            return False
        res = False
        for parent_type in self.types[type_to_check]:
            res = res or self._type_is_position(parent_type)
        return res

    def obj_is_position(self, object_name):
        if object_name in self.objects:
            obj_types = self.objects[object_name]
        else:
            obj_types = self._temp_objects[object_name]
        for obj_type in obj_types:
            if self._type_is_position(obj_type):
                return True
        return False

    # ----- Handling temporary goals, e.g. for exploration ---------------------

    def set_temp_goals(self, goal_list):
        self._temp_goals = goal_list

    def add_temp_object(self, object_type, object_name=None, object_value=None):
        assert object_type in self.types
        if object_name is not None:
            assert object_name not in self._temp_objects
            assert object_name not in self.lookup_table
        else:
            counter = 1
            while True:
                object_name = "{}_sample_{}".format(object_type, counter)
                if (
                    object_name not in self.objects
                    and object_name not in self._temp_objects
                ):
                    break
                counter += 1
        self._temp_objects[object_name] = [object_type]
        if object_value is not None:
            self.lookup_table[object_name] = object_value
        return object_name

    def make_permanent(self, obj_name):
        self.objects[obj_name] = self._temp_objects[obj_name]
        del self._temp_objects[obj_name]

    def solve_temp(self):
        self.pddl_if_temp.write_domain_pddl(self.actions, self._predicates, self.types)
        objects = deepcopy(self._temp_objects)
        objects.update(self.objects)
        self.pddl_if_temp.write_problem_pddl(
            objects, self.initial_predicates, self._temp_goals
        )
        return planner_interface.pddl_planner(
            self.pddl_if_temp._domain_file_pddl, self.pddl_if_temp._problem_file_pddl
        )

    def clear_temp(self):
        for obj in self._temp_objects:
            if obj in self.lookup_table:
                del self.lookup_table[obj]
        self._temp_objects.clear()
        del self._temp_goals[:]
