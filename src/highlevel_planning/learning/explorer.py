# Imports
import numpy as np
from copy import deepcopy
import pybullet as p

from highlevel_planning.knowledge.problem import PlanningProblem
from highlevel_planning.pddl_interface import pddl_file_if, planner_interface
from highlevel_planning.execution.es_sequential_execution import SequentialExecution
from highlevel_planning.tools.util import get_combined_aabb

# Parameters
# TODO: move them to config file

max_samples_per_seq_len = 50
max_failed_samples = 50

bounding_box_inflation_length = 0.5

# ------------------------------------------------------


class Explorer:
    def __init__(
        self,
        pddl_if,
        planning_problem,
        skill_set,
        knowledge_lookups,
        robot_uid,
        scene_objects,
    ):
        self.pddl_if_main = pddl_if
        self.action_list = [act for act in pddl_if._actions]
        self.planning_problem = planning_problem
        self.skill_set = skill_set
        self.knowledge_lookups = knowledge_lookups
        self.robot_uid_ = robot_uid
        self.scene_objects = scene_objects

    def exploration(self, predicates):
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

        count_seq_found = [0] * 4
        count_preplan_plan_success = [0] * 4
        count_preplan_run_success = [0] * 4
        count_seq_run_success = [0] * 4
        count_goal_reached = [0] * 4

        # Sample action sequences until a successful one was found
        while True:
            # Iterate through action sequence lengths
            for seq_len in range(1, 4):
                print("----- Sequence length: {} ----------".format(seq_len))
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

                    # try:
                    #     find_idx = sequence.index("place")
                    #     if parameter_samples[find_idx]["obj"] == "cube1":
                    #         print("hey")
                    # except ValueError:
                    #     pass

                    count_seq_found[seq_len - 1] += 1

                    # Found a feasible action sequence. Now test it.
                    # print("------------------------------------------")
                    # print("Sequence: " + str(seq))
                    # print("Params: " + str(params))
                    # print("Preconds: " + str(sequence_preconds))

                    # Set up planning problem that takes us to state where all preconditions are met
                    problem_preplan = deepcopy(self.planning_problem)
                    problem_preplan.goals = sequence_preconds
                    problem_preplan.populate_objects(
                        knowledge_lookups=self.knowledge_lookups
                    )

                    pddl_if.clear_planning_problem()
                    pddl_if.add_planning_problem(problem_preplan)
                    pddl_if.write_domain_pddl()
                    pddl_if.write_problem_pddl()

                    plan = planner_interface.pddl_planner(
                        pddl_if._domain_file_pddl, pddl_if._problem_file_pddl
                    )

                    if plan is not False:
                        count_preplan_plan_success[seq_len - 1] += 1

                        print("============")
                        print("Preplan:")
                        print(plan)

                        # Restore initial state
                        p.restoreState(stateId=current_state_id)

                        # Execute plan to get to start of sequence
                        success = self._execute_plan(plan)
                        if not success:
                            continue
                        print("Preplan SUCCESS")
                        count_preplan_run_success[seq_len - 1] += 1

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

                        print("----------")
                        print("Sequence: ")
                        for act in sequence_plan:
                            print(act)

                        # Useful for debugging:
                        # if (
                        #     sequence[0] == "place"
                        #     and parameter_samples[0]["obj"] == "cube1"
                        # ):
                        #     print("hey")

                        success = self._execute_plan(sequence_plan)
                        if not success:
                            continue

                        print("Sequence SUCCESS")
                        count_seq_run_success[seq_len - 1] += 1

                        # Check if the goal was reached
                        success = self.planning_problem.test_goals(predicates)
                        if not success:
                            continue
                        print("GOAL REACHED!!!")
                        count_goal_reached[seq_len - 1] += 1

            break

        print("===============================================================")
        print("count_seq_found")
        print(count_seq_found)
        print("count_preplan_plan_success")
        print(count_preplan_plan_success)
        print("count_preplan_run_success")
        print(count_preplan_run_success)
        print("count_seq_run_success")
        print(count_seq_run_success)
        print("count_goal_reached")
        print(count_goal_reached)

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
            try:
                params, params_tuple = self._sample_parameters(seq)
            except NameError:
                continue
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

        types_by_parent = self.pddl_if_main.get_types_by_parent_type()

        for idx_action, action in enumerate(sequence):
            parameter_samples[idx_action] = dict()
            parameters_current_action = list()
            for parameter in self.pddl_if_main._actions[action]["params"]:
                obj_type = parameter[1]
                obj_name = parameter[0]

                if obj_type == "position":
                    position = self._sample_position()
                    obj_sample = self.knowledge_lookups["position"].add(position)
                else:
                    if obj_type in types_by_parent:
                        objects_to_sample_from = [
                            objects_of_interest[sub_type]
                            for sub_type in types_by_parent[obj_type]
                        ]
                        try:
                            objects_to_sample_from.append(objects_of_interest[obj_type])
                        except KeyError:
                            pass
                        objects_to_sample_from = [
                            item
                            for sublist in objects_to_sample_from
                            for item in sublist
                        ]
                    else:
                        objects_to_sample_from = objects_of_interest[obj_type]

                    if len(objects_to_sample_from) == 0:
                        # No object of the desired type exists, sample new sequence
                        raise NameError(
                            "No object of desired type among objects of interest"
                        )
                    obj_sample = np.random.choice(objects_to_sample_from)
                parameter_samples[idx_action][obj_name] = obj_sample

                parameters_current_action.append(obj_sample)
            parameter_samples_tuples[idx_action] = tuple(parameters_current_action)

        return parameter_samples, parameter_samples_tuples

    def _sample_position(self):
        # Choose one goal object next to which to sample
        objects_goal = self._get_items_goal(objects_only=True)
        obj_sample = np.random.choice(objects_goal)
        uid = self.scene_objects[obj_sample].model.uid

        # Get AABB
        bounding_box = get_combined_aabb(uid)

        # Inflate the bounding box
        min_coords = bounding_box[0]
        max_coords = bounding_box[1]
        max_coords += bounding_box_inflation_length
        min_coords -= bounding_box_inflation_length
        min_coords[2] = np.max([min_coords[2], 0.0])

        # Sample
        sample = np.random.uniform(low=min_coords, high=max_coords)
        return sample

    def _get_objects_of_interest(self):
        # TODO finish and use this function
        interest_locations = list()

        # Get robot position
        temp = p.getBasePositionAndOrientation(self.robot_uid_)
        robot_pos = np.array(temp[0])
        interest_locations.append(robot_pos)

        # Get positions of objects that are in the goal description
        for goal in self.planning_problem.goals:
            for arg in goal[2]:
                if arg in self.scene_objects:
                    pos = p.getBasePositionAndOrientation(
                        self.scene_objects[arg].model.uid
                    )
                    interest_locations.append(np.array(pos))
                elif arg in self.knowledge_lookups["position"].data:
                    interest_locations.append(
                        self.knowledge_lookups["position"].get(arg)
                    )

        # Add scene objects that are close to interest locations

    def _get_items_goal(self, objects_only=False):
        """
        Get objects that involved in the goal description
        """
        item_list = list()
        for goal in self.planning_problem.goals:
            for arg in goal[2]:
                if objects_only:
                    if arg in self.scene_objects:
                        item_list.append(arg)
                else:
                    item_list.append(arg)
        return item_list

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
