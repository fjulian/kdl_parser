# Imports
import numpy as np

# Parameters
# TODO: move them to config file

num_samples_per_length = 20

# ------------------------------------------------------


class Explorer:
    def __init__(self, pddl_if, planning_problem):
        self.pddl_if = pddl_if
        self.action_list = [act for act in pddl_if._actions]
        self.planning_problem = planning_problem

    def exploration(self):
        # Some useful variables
        num_actions = len(self.pddl_if._actions)

        # Identify objects that are involved in reaching the goal
        relevant_objects = []
        for goal in self.planning_problem.goals:
            relevant_objects.extend(goal[2])

        # Sample action sequences until a successful one was found
        while True:
            # Iterate through action sequence lengths
            for seq_len in range(1, 5):
                seq = self.sample_sequence(seq_len)

            #     action_seqs = list(range(num_actions))
            # random_action_idx = np.random.randint

    def sample_sequence(self, length):
        # Generate the sequence
        sequence = []
        for _ in range(length):
            while True:
                temp = np.random.choice(self.action_list)
                if not temp in sequence:
                    sequence.append(temp)
                    break
        return sequence

    def sample_parameters(self, sequence):
        # Create list of relevant items in the scene
        
        for action in sequence:
            for 

    def test_abstract_feasibility(self):
        # Check of the sequence is feasible on the abstract level
        facts = set()
        sequence_invalid = False
        for action_id in sequence:
            action_descr = self.pddl_if._actions[action_id]

            # Check if any fact contradicts the pre-conditions of this action
            for fact in facts:
                for effect in action_descr["effects"]:
                    if fact[0] == effect[0] and fact[2] == effect[2]:
                        if not fact[1] == effect[1]:
                            sequence_invalid = True
                            break
                if sequence_invalid:
                    break

            if sequence_invalid:
                break

            for effect in action_descr["effects"]:
                for fact in facts:
                    if fact[0] == effect[0] and fact[2] == effect[2]:
                        facts.remove(fact)
                facts.add(effect)
