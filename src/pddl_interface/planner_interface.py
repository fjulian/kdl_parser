import subprocess

class PDDLsolver:
    def __init__(self):
        pass

    def plan(self):
        res = subprocess.check_output(['bin/ff', '-s', '2', '-o', 'knowledge/pddl-examples/rover/strips_typing/domain.pddl', '-f', 'knowledge/pddl-examples/rover/strips_typing/problem.pddl'])
        # res = subprocess.check_output(['bin/ff', '-s', '2', '-o', 'knowledge/pddl-examples/rover/strips/domain.pddl', '-f', 'knowledge/pddl-examples/rover/strips/problem.pddl'])
        res = self.cut_string_before(res, "ff: found legal plan as follows")
        res = self.cut_string_before(res, "0:")
        res = self.cut_string_at(res, "time spent")
        res = res.split('\n')
        for i in range(len(res)):
            res[i] = res[i].strip().lower()
        while True:
            try:
                res.remove('')
            except ValueError:
                break
        # print(res)
        return res

    def cut_string_before(self, string, query):
        # Finds query in string and cuts everything before it.
        start_idx = string.find(query)
        if start_idx > -1:
            string = string[start_idx:]
        return string

    def cut_string_at(self, string, query):
        # Finds query in string and cuts it away, together with everything that comes behind. 
        start_idx = string.find(query)
        if start_idx > -1:
            string = string[:start_idx]
        return string
