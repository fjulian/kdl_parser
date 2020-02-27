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
            for seq_len in range(2, 5):
                # Sample sequences until a abstractly feasible one was found
                while True:
                    seq = self._sample_sequence(seq_len)
                    params = self._sample_parameters(seq)
                    if self._test_abstract_feasibility(seq, params):
                        break

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
            parameter_samples[idx_action] = dict()
            for parameter in self.pddl_if._actions[action]["params"]:
                obj_type = parameter[1]
                obj_name = parameter[0]
                obj_sample = np.random.choice(objects_of_interest[obj_type])
                parameter_samples[idx_action][obj_name] = obj_sample

        return parameter_samples

    def _test_abstract_feasibility(self, sequence, parameters):
        """
        Takes an action sequence and suitable parameters as inputs and checks
        whether the sequence is logically feasible.
        
        Args:
            sequence (list): The action sequence
            parameters (list): Parameters for each action
        
        Returns:
            bool: True if the sequence is feasible, False otherwise.
        """

        facts = list()
        sequence_invalid = False
        for action_idx, action_id in enumerate(sequence):
            action_descr = self.pddl_if._actions[action_id]

            # Check if any fact contradicts the pre-conditions of this action
            for fact in facts:
                for precond in action_descr["preconds"]:
                    parametrized_precond = (
                        precond[0],
                        precond[1],
                        [parameters[action_idx][obj_name] for obj_name in precond[2]],
                    )
                    if (
                        fact[0] == parametrized_precond[0]
                        and fact[2] == parametrized_precond[2]
                    ):
                        if not fact[1] == parametrized_precond[1]:
                            sequence_invalid = True
                            break
                if sequence_invalid:
                    break

            if sequence_invalid:
                break

            for effect in action_descr["effects"]:
                parametrized_effect = (
                    effect[0],
                    effect[1],
                    [parameters[action_idx][obj_name] for obj_name in effect[2]],
                )
                for fact in facts:
                    if (
                        fact[0] == parametrized_effect[0]
                        and fact[2] == parametrized_effect[2]
                    ):
                        facts.remove(fact)
                facts.append(parametrized_effect)
        return not sequence_invalid
