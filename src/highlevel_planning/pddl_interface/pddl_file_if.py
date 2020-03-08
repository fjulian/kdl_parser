from datetime import datetime
import json
from os import path, mkdir
import pickle


class PDDLFileInterface:
    def __init__(
        self, domain_dir, problem_dir=None, initial_domain_pddl=None, domain_name=""
    ):
        # Some folder book keeping
        self._domain_dir = domain_dir
        if problem_dir is None:
            self._problem_dir = domain_dir
        else:
            self._problem_dir = problem_dir
        if not path.isdir(self._domain_dir):
            mkdir(self._domain_dir)
        if not path.isdir(self._problem_dir):
            mkdir(self._problem_dir)

        # Domain definition
        self._domain_name = domain_name
        self._predicates = {}
        self._actions = {}
        self._types = list()

        # Problem definition
        self._objects = []
        self._initial_predicates = []
        self._goals = []

        self._domain_file = path.join(self._domain_dir, "_domain.pkl")
        self._domain_file_pddl = None
        self._problem_file_pddl = None
        if initial_domain_pddl is not None:
            self._domain_file_pddl = path.join(self._domain_dir, initial_domain_pddl)
            self.read_domain_pddl()
        else:
            self.load_domain()

        self._requirements = ":strips :typing"

        time_now = datetime.now()
        self._time_now_str = time_now.strftime("%y%m%d-%H%M%S")

    # ----- Loading and saving pickle with domain and problem ---------------------------

    def load_domain(self):
        print("Trying to load domain file...")
        if path.exists(self._domain_file):
            with open(self._domain_file, "rb") as f:
                load_obj = pickle.load(f)
            self._domain_name = load_obj[0]
            self._predicates = load_obj[1]
            self._actions = load_obj[2]
            print("Trying to load domain file... DONE")
        else:
            print("Trying to load domain file... NOT FOUND --> starting from scratch")

    def save_domain(self):
        save_obj = (self._domain_name, self._predicates, self._actions)
        with open(self._domain_file, "wb") as f:
            pickle.dump(save_obj, f)
        print("Saved domain file")

    # ----- Loading and saving PDDL files --------------------------------------

    def write_domain_pddl(self):
        all_types_present = self.check_types()
        if not all_types_present:
            raise ValueError("Not all types were defined properly")

        # Sort types by parent type
        types = dict()
        for type_item in self._types:
            if type_item[1] not in types:
                types[type_item[1]] = list()
            types[type_item[1]].append(type_item[0])

        pddl_str = ""
        pddl_str += "(define (domain " + self._domain_name + ")\n"
        pddl_str += "\t(:requirements " + self._requirements + ")\n\n"

        pddl_str += "\t(:types\n"
        if None in types:
            pddl_str += "\t\t"
            for type_item in types[None]:
                pddl_str += type_item + " "
            pddl_str += "- object\n"
        for parent_type in types:
            if parent_type is None:
                continue
            pddl_str += "\t\t"
            for type_item in types[parent_type]:
                pddl_str += type_item + " "
            pddl_str += "- " + parent_type + "\n"
        pddl_str += "\t)\n\n"

        pddl_str += "\t(:predicates\n"
        for pred in self._predicates:
            pddl_str += "\t\t(" + pred
            for item in self._predicates[pred]:
                pddl_str += " ?" + item[0] + " - " + item[1]
            pddl_str += ")\n"
        pddl_str += "\t)\n\n"
        for act in self._actions:
            pddl_str += "\t(:action " + act + "\n"
            pddl_str += "\t\t:parameters\n"
            pddl_str += "\t\t\t("
            for item in self._actions[act]["params"]:
                pddl_str += "?" + item[0] + " - " + item[1] + " "
            pddl_str = pddl_str[:-1]
            pddl_str += ")\n\n"
            pddl_str += "\t\t:precondition\n\t\t\t(and\n"
            for item in self._actions[act]["preconds"]:
                pddl_str += "\t\t\t\t("
                if not item[1]:
                    pddl_str += "not ("
                pddl_str += item[0]
                for param in item[2]:
                    pddl_str += " ?" + param
                if not item[1]:
                    pddl_str += ")"
                pddl_str += ")\n"
            pddl_str += "\t\t\t)\n\n"
            pddl_str += "\t\t:effect\n\t\t\t(and\n"
            for item in self._actions[act]["effects"]:
                pddl_str += "\t\t\t\t("
                if not item[1]:
                    pddl_str += "not ("
                pddl_str += item[0]
                for param in item[2]:
                    pddl_str += " ?" + param
                if not item[1]:
                    pddl_str += ")"
                pddl_str += ")\n"
            pddl_str += "\t\t\t)\n\n"
            pddl_str += "\t)\n\n"
        pddl_str += ")"

        new_filename = path.join(self._domain_dir, self._time_now_str + "_domain.pddl")
        with open(new_filename, "w") as f:
            f.write(pddl_str)
        # print("Wrote new PDDL domain file: " + new_filename.split("/")[-1])
        self._domain_file_pddl = new_filename

    def read_domain_pddl(self):
        with open(self._domain_file_pddl, "r") as f:
            dom = f.read()
        dom = dom.split("\n")

        predicates = {}

        # Parse predicates
        while len(dom) > 0:
            curr = dom.pop(0)
            curr = curr.strip()
            if curr.find("(define") > -1:
                splitted = curr.split(" ")
                self._domain_name = splitted[-1][:-1]
            elif curr.find("(:predicates") > -1:
                while True:
                    sub_curr = dom.pop(0)
                    sub_curr = sub_curr.strip()
                    if sub_curr == ")":
                        break
                    splitted = sub_curr.split(" ")
                    predicate = splitted.pop(0)[1:]
                    params = []
                    params_typed_idx = 0
                    while len(splitted) > 0:
                        param = splitted.pop(0)
                        param = param.replace(")", "")
                        if param == "-":
                            this_type = splitted.pop(0)
                            this_type = this_type.replace(")", "")
                            for i in range(len(params[params_typed_idx:])):
                                params[params_typed_idx + i][1] = this_type
                            params_typed_idx = len(params)
                        else:
                            param = param.replace("?", "")
                            params.append([param, None])
                    predicates[predicate] = params
                self._predicates = predicates
            elif curr.find("(:action") > -1:
                action = curr.split(" ")[1]
                while True:
                    sub_curr = dom.pop(0)
                    sub_curr = sub_curr.strip()
                    if sub_curr == ")":
                        break
                    elif sub_curr.find(":parameters") > -1:
                        sub_curr = dom.pop(0)
                        sub_curr = sub_curr.strip()
                        splitted = sub_curr.split(" ")
                        params = []
                        params_typed_idx = 0
                        while len(splitted) > 0:
                            param = splitted.pop(0)
                            param = param.replace("(", "")
                            param = param.replace(")", "")
                            if param == "-":
                                this_type = splitted.pop(0)
                                this_type = this_type.replace(")", "")
                                for i in range(len(params[params_typed_idx:])):
                                    params[params_typed_idx + i][1] = this_type
                                params_typed_idx = len(params)
                            else:
                                param = param.replace("?", "")
                                params.append([param, None])
                    elif sub_curr.find(":precondition") > -1:
                        sub_curr = dom.pop(0).strip()
                        preconds = []
                        if sub_curr == "(and":
                            sub_curr = dom.pop(0).strip()
                        while len(sub_curr) > 0:
                            if sub_curr == ")":
                                break
                            sub_curr = sub_curr.replace("(", "")
                            sub_curr = sub_curr.replace(")", "")
                            splitted = sub_curr.split(" ")
                            first_token = splitted.pop(0)
                            if first_token == "not":
                                negated = True
                                precond_name = splitted.pop(0)
                            else:
                                negated = False
                                precond_name = first_token
                            assert precond_name in list(self._predicates.keys())
                            precond_params = []
                            while len(splitted) > 0:
                                param = splitted.pop(0)
                                param = param.replace("?", "")
                                precond_params.append(param)
                            preconds.append((precond_name, not negated, precond_params))
                            sub_curr = dom.pop(0).strip()
                    elif sub_curr.find(":effect") > -1:
                        sub_curr = dom.pop(0).strip()
                        effects = []
                        if sub_curr == "(and":
                            sub_curr = dom.pop(0).strip()
                        while len(sub_curr) > 0:
                            if sub_curr == ")":
                                break
                            sub_curr = sub_curr.replace("(", "")
                            sub_curr = sub_curr.replace(")", "")
                            splitted = sub_curr.split(" ")
                            first_token = splitted.pop(0)
                            if first_token == "not":
                                negated = True
                                effect_name = splitted.pop(0)
                            else:
                                negated = False
                                effect_name = first_token
                            assert effect_name in list(self._predicates.keys())
                            effect_params = []
                            while len(splitted) > 0:
                                param = splitted.pop(0)
                                param = param.replace("?", "")
                                effect_params.append(param)
                            effects.append((effect_name, not negated, effect_params))
                            sub_curr = dom.pop(0).strip()
                self._actions[action] = {
                    "params": params,
                    "preconds": preconds,
                    "effects": effects,
                }
        print("Read PDDL domain file")

    def write_problem_pddl(self):
        pddl_str = ""
        pddl_str += "(define (problem chimera-auto-problem)\n"

        pddl_str += "\t(:domain\n"
        pddl_str += "\t\t" + self._domain_name + "\n"
        pddl_str += "\t)\n\n"

        pddl_str += "\t(:objects\n"
        for obj in self._objects:
            pddl_str += "\t\t" + obj[0] + " - " + obj[1] + "\n"
        pddl_str += "\t)\n\n"

        if len(self._initial_predicates) > 0:
            pddl_str += "\t(:init\n"
            for init in self._initial_predicates:
                pddl_str += "\t\t("
                for it in init:
                    pddl_str += it + " "
                pddl_str = pddl_str[:-1]
                pddl_str += ")\n"
            pddl_str += "\t)\n\n"

        pddl_str += "\t(:goal\n"
        pddl_str += "\t\t(and\n"
        for g in self._goals:
            pddl_str += "\t\t\t("
            if not g[1]:
                # Negated
                pddl_str += "not ("
            pddl_str += g[0] + " "
            for it in g[2]:
                pddl_str += it + " "
            pddl_str = pddl_str[:-1]
            if not g[1]:
                pddl_str += ")"
            pddl_str += ")\n"
        pddl_str += "\t\t)\n"
        pddl_str += "\t)\n"

        pddl_str += ")\n"

        new_filename = path.join(
            self._problem_dir, self._time_now_str + "_problem.pddl"
        )
        with open(new_filename, "w") as f:
            f.write(pddl_str)
        # print("Wrote new PDDL problem file: " + new_filename.split("/")[-1])
        self._problem_file_pddl = new_filename

    # ----- Adding to the domain description ------------------------------------

    def add_action(self, action_name, action_definition, overwrite=False):
        if not overwrite and action_name in self._actions:
            print(
                "Action "
                + action_name
                + " already exists and no overwrite was requested. Ignoring request."
            )
        else:
            assert isinstance(action_name, str)
            self._actions[action_name] = action_definition

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
        for type_name in self._types:
            if type_name[0] == new_type:
                raise ValueError("Type name already exists")
        self._types.append((new_type, parent_type))

    # ----- Adding to the problem description ----------------------------------

    def add_objects(self, object_list):
        self._objects += object_list

        # Remove duplicates
        self._objects = list(dict.fromkeys(self._objects))

    def add_inital_predicates(self, pred_list):
        self._initial_predicates += pred_list

        # Remove duplicates
        self._initial_predicates = list(dict.fromkeys(self._initial_predicates))

    def add_goal(self, goal_list):
        self._goals += goal_list

        # Remove duplicates
        self._goals = list(dict.fromkeys(self._goals))

    def add_planning_problem(self, planning_problem):
        self.add_objects(planning_problem.objects)
        self.add_inital_predicates(planning_problem.initial_predicates)
        self.add_goal(planning_problem.goals)

    def clear_planning_problem(self):
        del self._objects[:]
        del self._initial_predicates[:]
        del self._goals[:]

    # ----- Helper functions ---------------------------------------------------

    def check_types(self):
        # Makes sure that all type were defined
        types = [item[0] for item in self._types]
        for pred in self._predicates:
            for item in self._predicates[pred]:
                if item[1] not in types:
                    print("The following type was not pre-defined: {}".format(item[1]))
                    return False
        for act in self._actions:
            for item in self._actions[act]["params"]:
                if item[1] not in types:
                    print("The following type was not pre-defined: {}".format(item[1]))
                    return False
        return True


### Assumed conventions:
# All arguments of a predicate are on the same line as the predicate name. Each line defines one predicate.
# For actions, all parameters are in the same line, starting below the :parameters keyword.
# For preconditions and effects, one is defined per line, starting on the line after "(and".
# After preconditions and effects, a blank line is expected.
# Lines starting with ';' are comments.
