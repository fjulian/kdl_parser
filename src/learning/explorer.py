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
                seq = self._sample_sequence(seq_len)
                params = self._sample_parameters(seq)

            #     action_seqs = list(range(num_actions))
            # random_action_idx = np.random.randint

    def _sample_sequence(self, length):
        # Generate the sequence
        sequence = []
        for _ in range(length):
            while True:
                temp = np.random.choice(self.action_list)
                if not temp in sequence:
                    sequence.append(temp)
                    break
        return sequence

    def _sample_parameters(self, sequence):
        parameter_samples = [None] * len(sequence)

        # Create list of relevant items in the scene
        # TODO For now this is just adding all objects in the scene. Instead, just add objects
        # currently in proximity to the robot and the objects of interest.
        objects_of_interest_list = self.planning_problem.objects

        # Sort objects of interest by type into a dictionary
        objects_of_interest = dict()
        for obj in objects_of_interest_list:
            if obj[1] in objects_of_interest:
                objects_of_interest[obj[1]].append(obj[0])
            else:
                objects_of_interest[obj[1]] = [obj[0]]

        for idx_action, action in enumerate(sequence):
            parameter_samples[idx_action] = list()
            for parameter in self.pddl_if._actions[action]["params"]:
                obj_type = parameter[1]
                obj_sample = np.random.choice(objects_of_interest[obj_type])
                parameter_samples[idx_action].append(obj_sample)

        return parameter_samples

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
