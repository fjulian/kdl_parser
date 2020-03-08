# Imports
import numpy as np
from copy import deepcopy
import pybullet as p

from highlevel_planning.knowledge.problem import PlanningProblem
from highlevel_planning.pddl_interface import pddl_file_if, planner_interface
from highlevel_planning.execution.es_sequential_execution import SequentialExecution

# Parameters
# TODO: move them to config file

max_samples_per_seq_len = 50
max_failed_samples = 50

# ------------------------------------------------------


class Explorer:
    def __init__(self, pddl_if, planning_problem, skill_set, knowledge_lookups):
        self.pddl_if_main = pddl_if
        self.action_list = [act for act in pddl_if._actions]
        self.planning_problem = planning_problem
        self.skill_set = skill_set
        self.knowledge_lookups = knowledge_lookups

    def exploration(self):
        # Some useful variables
        # num_actions = len(self.pddl_if_main._actions)

        # Save the state the robot is currently in
        current_state_id = p.saveState()

        # Identify objects that are involved in reaching the goal
        relevant_objects = list()
        for goal in self.planning_problem.goals:
            relevant_objects.extend(goal[2])

        # Start PDDL file interface
        pddl_if = pddl_file_if.PDDLFileInterface(
            domain_dir="knowledge/chimera/explore/domain",
            problem_dir="knowledge/chimera/explore/problem",
            domain_name="chimera-domain",
        )
        pddl_if._actions = self.pddl_if_main._actions
        pddl_if._predicates = self.pddl_if_main._predicates
        pddl_if._types = self.pddl_if_main._types

        # Sample action sequences until a successful one was found
        while True:
            # Iterate through action sequence lengths
            for seq_len in range(1, 5):
                print("----- Sequence length: {} ----------".format(seq_len))
                count_plan_successful = 0
                sequences_tried = set()
                for _ in range(max_samples_per_seq_len):
                    # Sample sequences until an abstractly feasible one was found
                    (
                        success,
                        sequence,
                        parameter_samples,
                        sequence_preconds,
                    ) = self._sample_feasible_sequence(seq_len, sequences_tried)

                    if not success:
                        print(
                            "Sampling failed. Abort searching in this sequence length."
                        )
                        break

                    # Found a feasible action sequence. Now test it.
                    # print("------------------------------------------")
                    # print("Sequence: " + str(seq))
                    # print("Params: " + str(params))
                    # print("Preconds: " + str(sequence_preconds))

                    # Set up planning problem that takes us to state where all preconditions are met
                    problem_preplan = deepcopy(self.planning_problem)
                    problem_preplan.goals = sequence_preconds

                    pddl_if.clear_planning_problem()
                    pddl_if.add_planning_problem(problem_preplan)
                    pddl_if.write_domain_pddl()
                    pddl_if.write_problem_pddl()

                    plan = planner_interface.pddl_planner(
                        pddl_if._domain_file_pddl, pddl_if._problem_file_pddl
                    )

                    if plan is not False:
                        count_plan_successful += 1

                        # Restore initial state
                        p.restoreState(stateId=current_state_id)

                        # Execute plan to get to start of sequence
                        self._execute_plan(plan)

                        # Execute sequence
                        sequence_plan = list()
                        for idx_action, action in enumerate(sequence):
                            act_string = str(idx_action) + ": " + action
                            for parameter in self.pddl_if_main._actions[action][
                                "params"
                            ]:
                                act_string += (
                                    " " + parameter_samples[idx_action][parameter[0]]
                                )
                            sequence_plan.append(act_string)
                        success = self._execute_plan(sequence_plan)
                        if not success:
                            continue

                        print("Successful plan execution: ")
                        for act in sequence_plan:
                            print(act)

                        # Check if the goal was reached
                        # TODO do this check

                print(
                    "Successful plans for sequence length {}: {}".format(
                        seq_len, count_plan_successful
                    )
                )
            break

    def _execute_plan(self, plan):
        es = SequentialExecution(self.skill_set, plan, self.knowledge_lookups)
        es.setup()
        while True:
            success, plan_finished = es.step()
            # TODO if we run into a failure, check why this failure happened and adapt the PDDL if necessary
            if plan_finished or not success:
                break
        return success

    def _sample_feasible_sequence(self, sequence_length, sequences_tried):
        # Sample sequences until an abstractly feasible one was found
        failed_samples = 0
        success = True
        sequence_preconds = None
        while True:
            failed_samples += 1
            if failed_samples > max_failed_samples:
                success = False
                break

            seq = self._sample_sequence(sequence_length)
            params, params_tuple = self._sample_parameters(seq)
            if (tuple(seq), tuple(params_tuple),) in sequences_tried:
                continue
            sequences_tried.add((tuple(seq), tuple(params_tuple)))
            sequence_preconds = self._determine_sequence_preconds(seq, params)
            if self._test_abstract_feasibility(seq, params, sequence_preconds):
                break
        return success, seq, params, sequence_preconds

    def _sample_sequence(self, length):
        # Generate the sequence
        sequence = list()
        for _ in range(length):
            while True:
                temp = np.random.choice(self.action_list)
                if len(sequence) == 0:
                    sequence.append(temp)
                    break
                elif temp != sequence[-1]:
                    # This ensures that we don't sample the same action twice in a row
                    sequence.append(temp)
                    break
        return sequence

    def _sample_parameters(self, sequence):
        parameter_samples = [None] * len(sequence)
        parameter_samples_tuples = [None] * len(sequence)

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
            parameters_current_action = list()
            for parameter in self.pddl_if_main._actions[action]["params"]:
                obj_type = parameter[1]
                obj_name = parameter[0]
                obj_sample = np.random.choice(objects_of_interest[obj_type])
                parameter_samples[idx_action][obj_name] = obj_sample

                parameters_current_action.append(obj_sample)
            parameter_samples_tuples[idx_action] = tuple(parameters_current_action)

        return parameter_samples, parameter_samples_tuples

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
            action_descr = self.pddl_if_main._actions[action_id]

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
            action_descr = self.pddl_if_main._actions[action_id]

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
            tuple([action_parameters[obj_name] for obj_name in predicate[2]]),
        )
