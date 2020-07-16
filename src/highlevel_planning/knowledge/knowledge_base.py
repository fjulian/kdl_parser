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
        self.predicate_funcs = None

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

        # Problem definition
        self.objects = dict()
        self.visible_objects = set()
        self.initial_predicates = list()
        # self.goals = list()
        # self.goals = [("in-hand", True, ("cube1", "robot1"))]
        # self.goals = [("at", True, ("container1", "robot1"))]
        # self.goals = [("at", True, ("cupboard", "robot1"))]
        # self.goals = [("on", True, ("cupboard", "cube1"))]
        # self.goals = [
        #     ("on", True, ("cupboard", "cube1")),
        #     ("at", True, ("container1", "robot1")),
        # ]
        self.goals = [("on", True, ("container2", "cube1"))]
        # self.goals = [("inside", True, ("container1", "cube1"))]

        # Value lookups (e.g. for positions)
        self.lookup_table = dict()
        self.parameterizations = dict()
        # Meta action info
        self.meta_actions = dict()

        # Load previous knkowledge base
        self._domain_file = path.join(domain_dir, "_domain.pkl")
        self.load_domain()

        # PDDL file interfaces
        self.pddl_if = PDDLFileInterface(domain_dir, problem_dir, domain_name)
        self.pddl_if_temp = PDDLFileInterface(
            temp_domain_dir, temp_problem_dir, domain_name
        )

        # Temporary variables (e.g. for exploration)
        self._temp_goals = list()
        self._temp_objects = dict()
        self._temp_initial_predicates = list()

    def set_predicate_funcs(self, preds):
        self.predicate_funcs = preds

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
            self.objects = load_obj[4]
            self.lookup_table = load_obj[5]
            self.parameterizations = load_obj[6]
            self.meta_actions = load_obj[7]
            print("Trying to load domain file... DONE")
        else:
            print("Trying to load domain file... NOT FOUND --> starting from scratch")

    def save_domain(self):
        save_obj = (
            self._domain_name,
            self._predicates,
            self.actions,
            self.types,
            self.objects,
            self.lookup_table,
            self.parameterizations,
            self.meta_actions,
        )
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
            # There can only be one parent per type
            ValueError("Type already exists")
        else:
            if parent_type is not None:
                assert parent_type in self.types
            self.types[new_type] = parent_type

    # ----- Adding to the problem description ----------------------------------

    def add_object(self, object_name, object_type, object_value=None):
        assert object_type in self.types
        if object_name in self.objects:
            if object_type not in self.objects[object_name]:
                self.objects[object_name].append(object_type)
                red_types = self._get_redundant_type(self.objects[object_name])
                for red_type in red_types:
                    if self.objects[object_name].index(red_type) != 0:
                        self.objects[object_name].remove(red_type)
        else:
            self.objects[object_name] = [object_type]
        if object_value is not None:
            self.lookup_table[object_name] = object_value
        self.visible_objects.add(object_name)

    def _get_redundant_type(self, object_types):
        redundant_types = set()
        for i, obj_type in enumerate(object_types):
            for potential_child in (x for j, x in enumerate(object_types) if j != i):
                if self.type_x_child_of_y(potential_child, obj_type):
                    redundant_types.add(obj_type)
                    break
        return list(redundant_types)

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
        self.pddl_if.write_pddl(self)
        return planner_interface.pddl_planner(
            self.pddl_if._domain_file_pddl, self.pddl_if._problem_file_pddl
        )

    # ----- Meta action handling -----------------------------------------------

    def expand_step(self, action_name, action_parameters):
        expanded_step = list()
        if action_name in self.meta_actions:
            meta_action = self.meta_actions[action_name]
            parameter_order = [
                param[0] for param in meta_action["description"]["params"]
            ]
            for idx, sub_action_name in enumerate(meta_action["seq"]):
                new_plan_item = [sub_action_name, []]
                sub_action_parameters = self.actions[sub_action_name]["params"]
                for param_spec in sub_action_parameters:
                    old_param_name = param_spec[0]
                    if old_param_name in meta_action["hidden_params"][idx]:
                        new_plan_item[1].append(meta_action["hidden_params"][idx][old_param_name])
                    elif old_param_name in meta_action["param_translator"][idx]:
                        new_param_name = meta_action["param_translator"][idx][
                            old_param_name
                        ]
                        parameter_idx = parameter_order.index(new_param_name)
                        parameter_value = action_parameters[parameter_idx]
                        new_plan_item[1].append(parameter_value)
                    else:
                        raise RuntimeError(
                            "Parameter for sub action of meta action undefined"
                        )
                expanded_step.append(new_plan_item)
        else:
            new_plan_item = [action_name, action_parameters]
            expanded_step.append(new_plan_item)
        return expanded_step

    def add_meta_action(
            self,
            name,
            sequence,
            parameters,
            param_translator,
            hidden_parameters,
            description,
    ):
        assert type(name) is str
        assert type(sequence) is list
        self.meta_actions[name] = {
            "seq": sequence,
            "params": parameters,
            "param_translator": param_translator,
            "hidden_params": hidden_parameters,
            "description": description,
        }

    # ----- Utilities ----------------------------------------------------------

    def test_goals(self):
        for goal in self.goals:
            if not self.predicate_funcs.call[goal[0]](*goal[2]) == goal[1]:
                return False
        return True

    def check_predicates(self):
        """
        If predicates need to be initialized when the system is launched, this can be done here.
        """

        if self.predicate_funcs.empty_hand("robot1"):
            self.initial_predicates.append(("empty-hand", "robot1"))
        self.initial_predicates.append(("in-reach", "origin", "robot1"))
        self.initial_predicates.append(("at", "origin", "robot1"))

        # Check any predicates in relation with the goal
        for goal in self.goals:
            if self.predicate_funcs.call[goal[0]](*goal[2]):
                pred_tuple = (goal[0],) + goal[2]
                self.initial_predicates.append(pred_tuple)

    def populate_visible_objects(self, scene):
        # TODO maybe move this into a separate dummy perception module
        for obj in scene.objects:
            self.add_object(obj, "item")
            if self.predicate_funcs.call["has-grasp"](obj):
                self.initial_predicates.append(("has-grasp", obj))

        # Add "objects" that are always visible
        for object_name in self.objects:
            for object_type in self.objects[object_name]:
                if self.type_x_child_of_y(object_type, "position"):
                    self.add_object(object_name, "position")
                    self.initial_predicates.append(("has-grasp", object_name))
                    break

    def type_x_child_of_y(self, x, y):
        parent_type = self.types[x]
        if x == y:
            return True
        elif parent_type is None:
            return False
        return self.type_x_child_of_y(parent_type, y)

    def is_type(self, object_to_check, type_query):
        if object_to_check in self.objects:
            obj_types = self.objects[object_to_check]
        else:
            obj_types = self._temp_objects[object_to_check]
        for obj_type in obj_types:
            if self.type_x_child_of_y(obj_type, type_query):
                return True
        return False

    def get_objects_by_type(
            self,
            type_query,
            types_by_parent,
            objects_by_type,
            object_set=None,
            visible_only=False,
    ):
        if object_set is None:
            object_set = set()
        if type_query in objects_by_type:
            for obj in objects_by_type[type_query]:
                if not visible_only or obj in self.visible_objects:
                    object_set.add(obj)
        if type_query in types_by_parent:
            for sub_type in types_by_parent[type_query]:
                object_set = self.get_objects_by_type(
                    sub_type,
                    types_by_parent,
                    objects_by_type,
                    object_set,
                    visible_only=visible_only,
                )
        return object_set

    # ----- Handling temporary goals, e.g. for exploration ---------------------

    def set_temp_goals(self, goal_list):
        self._temp_goals = deepcopy(goal_list)

    def add_temp_object(self, object_type, object_name=None, object_value=None):
        assert object_type in self.types
        if object_name is not None:
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
        if object_name in self._temp_objects:
            if object_type not in self._temp_objects[object_name]:
                self._temp_objects[object_name].append(object_type)
                red_types = self._get_redundant_type(self._temp_objects[object_name])
                for red_type in red_types:
                    if self._temp_objects[object_name].index(red_type) != 0:
                        self._temp_objects[object_name].remove(red_type)
        else:
            self._temp_objects[object_name] = [object_type]
            if self.is_type(object_name, "position"):
                self._temp_initial_predicates.append(("has-grasp", object_name))
        if object_value is not None:
            self.lookup_table[object_name] = object_value
        return object_name

    def generalize_temp_object(self, object_name):
        assert object_name in self.objects
        for new_type in self.types:
            self.add_temp_object(object_type=new_type, object_name=object_name)

    def make_permanent(self, obj_name):
        self.objects[obj_name] = self._temp_objects[obj_name]
        del self._temp_objects[obj_name]
        for pred in self._temp_initial_predicates:
            if obj_name in pred:
                self.initial_predicates.append(pred)

    def joined_objects(self):
        objects = deepcopy(self._temp_objects)
        for obj in self.objects:
            if obj in objects:
                objects[obj] = list(dict.fromkeys(objects[obj] + self.objects[obj]))
            else:
                objects[obj] = self.objects[obj]
        return objects

    def solve_temp(self):
        self.pddl_if_temp.write_domain_pddl(self.actions, self._predicates, self.types)
        objects = self.joined_objects()
        self.pddl_if_temp.write_problem_pddl(
            objects, self.initial_predicates + self._temp_initial_predicates, self._temp_goals
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
        del self._temp_initial_predicates[:]
