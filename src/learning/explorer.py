# Imports
import numpy as np
from copy import deepcopy

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
                    sequence_preconds = self._determine_sequence_preconds(seq, params)
                    if self._test_abstract_feasibility(seq, params, sequence_preconds):
                        break

                # Found a feasible action sequence. Now test it.

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

    def _test_abstract_feasibility(self, sequence, parameters, preconds):
        """
        Takes an action sequence and suitable parameters as inputs and checks
        whether the sequence is logically feasible.
        
        Args:
            sequence (list): The action sequence
            parameters (list): Parameters for each action
        
        Returns:
            bool: True if the sequence is feasible, False otherwise.
        """

        facts = deepcopy(preconds)
        sequence_invalid = False
        for action_idx, action_id in enumerate(sequence):
            action_descr = self.pddl_if._actions[action_id]

            # Check if any fact contradicts the pre-conditions of this action
            for fact in facts:
                for precond in action_descr["preconds"]:
                    parametrized_precond = self._parametrize_predicate(
                        precond, parameters[action_idx]
                    )
                    if (
                        fact[0] == parametrized_precond[0]
                        and fact[2] == parametrized_precond[2]
                        and not fact[1] == parametrized_precond[1]
                    ):
                        sequence_invalid = True
                        break
                if sequence_invalid:
                    break

            if sequence_invalid:
                break

            for effect in action_descr["effects"]:
                parametrized_effect = self._parametrize_predicate(
                    effect, parameters[action_idx]
                )
                facts_to_remove = list()
                for fact in facts:
                    if (
                        fact[0] == parametrized_effect[0]
                        and fact[2] == parametrized_effect[2]
                    ):
                        facts_to_remove.append(fact)
                for fact in facts_to_remove:
                    facts.remove(fact)
                facts.append(parametrized_effect)
        return not sequence_invalid

    def _determine_sequence_preconds(self, sequence, parameters):
        seq_preconds = list()
        for action_idx, action_id in reversed(list(enumerate(sequence))):
            action_descr = self.pddl_if._actions[action_id]

            # Remove all effects of this action from the precond list
            preconds_to_remove = list()
            for seq_precond in seq_preconds:
                for effect in action_descr["effects"]:
                    parametrized_effect = self._parametrize_predicate(
                        effect, parameters[action_idx]
                    )
                    if seq_precond == parametrized_effect:
                        preconds_to_remove.append(seq_precond)
            for seq_precond in preconds_to_remove:
                seq_preconds.remove(seq_precond)

            # Add all preconditions of this action to the precond list
            for precond in action_descr["preconds"]:
                parametrized_precond = self._parametrize_predicate(
                    precond, parameters[action_idx]
                )
                if parametrized_precond not in seq_preconds:
                    seq_preconds.append(parametrized_precond)
        return seq_preconds

    def _parametrize_predicate(self, predicate, action_parameters):
        return (
            predicate[0],
            predicate[1],
            [action_parameters[obj_name] for obj_name in predicate[2]],
        )
