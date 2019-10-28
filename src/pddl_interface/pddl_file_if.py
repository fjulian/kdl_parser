from datetime import datetime


class PDDLFileInterface:
    def __init__(self, domain_file=None, problem_file=None):
        self._domain_file = domain_file
        self._domain_dir = "knowledge/chimera/domain/"
        self._problem_file = problem_file

        self._requirements = ":strips :typing"

        self._domain_name = ""
        self._predicates = {}
        self._actions = {}
        self._types = []

    def write_domain(self):
        self.extract_types()

        pddl_str = ""
        pddl_str += "(define (domain " + self._domain_name + ")\n"
        pddl_str += "\t(:requirements " + self._requirements + ")\n\n"
        types_str = ' '
        types_str = types_str.join(self._types)
        pddl_str += "\t(:types " + types_str + ")\n\n"
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
                if item[1]:
                    pddl_str += "not ("
                pddl_str += item[0]
                for param in item[2]:
                    pddl_str += " ?" + param
                pddl_str += ")\n"
            pddl_str += "\t\t\t)\n\n"
            pddl_str += "\t\t:effect\n\t\t\t(and\n"
            for item in self._actions[act]["effects"]:
                pddl_str += "\t\t\t\t("
                if item[1]:
                    pddl_str += "not ("
                pddl_str += item[0]
                for param in item[2]:
                    pddl_str += " ?" + param
                if item[1]:
                    pddl_str += ")"
                pddl_str += ")\n"
            pddl_str += "\t\t\t)\n\n"
            pddl_str += "\t)\n\n"
        pddl_str += ")"
        
        time_now = datetime.now()
        new_filename = self._domain_dir + time_now.strftime("%y%m%d-%H%M%S_domain.pddl")
        with open(new_filename, 'w') as f:
            f.write(pddl_str)
        print("Wrote new domain file: "+new_filename.split('/')[-1])

    def extract_types(self):
        types = []
        for pred in self._predicates:
            for item in self._predicates[pred]:
                if item[1] not in types:
                    types.append(item[1])
        for act in self._actions:
            for item in self._actions[act]['params']:
                if item[1] not in types:
                    types.append(item[1])
        self._types = types

    def read_domain(self):
        with open(self._domain_file, 'r') as f:
            dom = f.read()
        dom = dom.split('\n')

        predicates = {}

        # Parse predicates
        while len(dom) > 0:
            curr = dom.pop(0)
            curr = curr.strip()
            if curr.find("(define") > -1:
                splitted = curr.split(' ')
                self._domain_name = splitted[-1][:-1]
            elif curr.find("(:predicates") > -1:
                while True:
                    sub_curr = dom.pop(0)
                    sub_curr = sub_curr.strip()
                    if sub_curr == ')':
                        break
                    splitted = sub_curr.split(' ')
                    predicate = splitted.pop(0)[1:]
                    params = []
                    params_typed_idx = 0
                    while len(splitted) > 0:
                        param = splitted.pop(0)
                        param = param.replace(')','')
                        if param == '-':
                            this_type = splitted.pop(0)
                            this_type = this_type.replace(')','')
                            for i in range(len(params[params_typed_idx:])):
                                params[params_typed_idx+i][1] = this_type
                            params_typed_idx = len(params)
                        else:
                            param = param.replace('?','')
                            params.append([param, None])
                    predicates[predicate] = params
                self._predicates = predicates
            elif curr.find("(:action") > -1:
                action = curr.split(' ')[1]
                while True:
                    sub_curr = dom.pop(0)
                    sub_curr = sub_curr.strip()
                    if sub_curr == ')':
                        break
                    elif sub_curr.find(':parameters') > -1:
                        sub_curr = dom.pop(0)
                        sub_curr = sub_curr.strip()
                        splitted = sub_curr.split(' ')
                        params = []
                        params_typed_idx = 0
                        while len(splitted) > 0:
                            param = splitted.pop(0)
                            param = param.replace('(','')
                            param = param.replace(')','')
                            if param == '-':
                                this_type = splitted.pop(0)
                                this_type = this_type.replace(')','')
                                for i in range(len(params[params_typed_idx:])):
                                    params[params_typed_idx+i][1] = this_type
                                params_typed_idx = len(params)
                            else:
                                param = param.replace('?','')
                                params.append([param, None])
                    elif sub_curr.find(":precondition") > -1:
                        sub_curr = dom.pop(0).strip()
                        preconds = []
                        if sub_curr == '(and':
                            sub_curr = dom.pop(0).strip()
                        while len(sub_curr) > 0:
                            sub_curr = sub_curr.replace('(','')
                            sub_curr = sub_curr.replace(')','')
                            splitted = sub_curr.split(' ')
                            first_token = splitted.pop(0)
                            if first_token == 'not':
                                negated = True
                                precond_name = splitted.pop(0)
                            else:
                                negated = False
                                precond_name = first_token
                            assert(precond_name in list(self._predicates.keys()))
                            precond_params = []
                            while len(splitted) > 0:
                                param = splitted.pop(0)
                                param = param.replace('?','')
                                precond_params.append(param)
                            preconds.append((precond_name, negated, precond_params))
                            sub_curr = dom.pop(0).strip()
                    elif sub_curr.find(":effect") > -1:
                        sub_curr = dom.pop(0).strip()
                        effects = []
                        if sub_curr == '(and':
                            sub_curr = dom.pop(0).strip()
                        while len(sub_curr) > 0:
                            sub_curr = sub_curr.replace('(','')
                            sub_curr = sub_curr.replace(')','')
                            splitted = sub_curr.split(' ')
                            first_token = splitted.pop(0)
                            if first_token == 'not':
                                negated = True
                                effect_name = splitted.pop(0)
                            else:
                                negated = False
                                effect_name = first_token
                            assert(effect_name in list(self._predicates.keys()))
                            effect_params = []
                            while len(splitted) > 0:
                                param = splitted.pop(0)
                                param = param.replace('?','')
                                effect_params.append(param)
                            effects.append((effect_name, negated, effect_params))
                            sub_curr = dom.pop(0).strip()
                self._actions[action] = {"params": params, "preconds": preconds, "effects": effects}
        print("Finished parsing file")


### Assumed conventions:
# All arguments of a predicate are on the same line as the predicate name. Each line defines one predicate.
# For actions, all parameters are in the same line, starting below the :parameters keyword.
# For preconditions and effects, one is defined per line, starting on the line after "(and".
# After preconditions and effects, a blank line is expected.