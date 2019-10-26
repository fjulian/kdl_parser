


class PDDLFileInterface:
    def __init__(self, domain_file=None, problem_file=None):
        self._domain_file = domain_file
        self._problem_file = problem_file

        self._domain_name = ""
        self._predicates = {}

    def write_domain(self):
        pass

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
                pass

                        

### Assumed conventions:
# All arguments of a predicate are on the same line as the predicate name. Each line defines one predicate.
# For actions, there is one parameter on each line, starting below the :parameters keyword. Each parameter has its type defined behind it.
# Same holds for preconditions and effects.
