import subprocess

def pddl_planner(domain_file, problem_file):
    try:
        res = subprocess.check_output(['bin/ff', '-s', '2', '-o', domain_file, '-f', problem_file])
        # res = subprocess.check_output(['bin/ff', '-s', '2', '-o', 'knowledge/pddl-examples/rover/strips/domain.pddl', '-f', 'knowledge/pddl-examples/rover/strips/problem.pddl'])
    except subprocess.CalledProcessError:
        print("Planning failed")
        return False
    try:
        res = cut_string_before(res, "ff: found legal plan as follows", complain=True)
    except NameError:
        print("Planning failed")
        return False
    res = cut_string_before(res, "0:")
    res = cut_string_at(res, "time spent")
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

def cut_string_before(string, query, complain=False):
    # Finds query in string and cuts everything before it.
    start_idx = string.find(query)
    if start_idx > -1:
        string = string[start_idx:]
    elif complain:
        raise NameError("Query not found")
    return string

def cut_string_at(string, query):
    # Finds query in string and cuts it away, together with everything that comes behind. 
    start_idx = string.find(query)
    if start_idx > -1:
        string = string[:start_idx]
    return string
