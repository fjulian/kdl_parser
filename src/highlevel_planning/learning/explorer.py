# Imports
import numpy as np
from copy import deepcopy
import pybullet as p

from highlevel_planning.execution.es_sequential_execution import SequentialExecution
from highlevel_planning.tools.util import get_combined_aabb
from highlevel_planning.learning.logic_tools import (
    parametrize_predicate,
    determine_sequence_preconds,
    test_abstract_feasibility,
    invert_dict,
)

# ----- Parameters -------------------------------------
# TODO: move them to config file

max_sample_repetitions = 2
max_seq_len = 2
max_samples_per_seq_len = 100
max_failed_samples = 50

bounding_box_inflation_length = 0.2

# ------------------------------------------------------


class Explorer:
    def __init__(
        self, skill_set, robot_uid, scene_objects, pddl_extender, knowledge_base,
    ):
        self.action_list = [act for act in knowledge_base.actions]
        self.skill_set = skill_set
        self.robot_uid_ = robot_uid
        self.scene_objects = scene_objects
        self.pddl_extender = pddl_extender
        self.knowledge_base = knowledge_base

        self.current_state_id = None

    def exploration(self):
        np.random.seed(0)
        sequences_tried = set()

        # Save the state the robot is currently in
        self.current_state_id = p.saveState()

        # Identify objects that are involved in reaching the goal
        relevant_objects = list()
        for goal in self.knowledge_base.goals:
            relevant_objects.extend(goal[2])

        res = self._explore_generalized_action(relevant_objects, sequences_tried)
        if not res:
            res = self._explore_goal_objects(relevant_objects, sequences_tried)
        if not res:
            res = self._explore_all(sequences_tried)
        return res

    # ----- Different sampling strategies ------------------------------------

    def _explore_generalized_action(self, relevant_objects, sequences_tried):

        # Check if an action with a similar effect already exists
        self.knowledge_base.clear_temp()
        for obj in relevant_objects:
            self.knowledge_base.generalize_temp_object(obj)
        self.knowledge_base.set_temp_goals(self.knowledge_base.goals)
        plan = self.knowledge_base.solve_temp()
        if not plan:
            return False

        # Extract parameters from plan
        fixed_parameters_full = [None] * len(plan)
        for plan_idx, plan_item in enumerate(plan):
            plan_item_list = plan_item.split(" ")
            action_name = plan_item_list[1]
            action_description = self.knowledge_base.actions[action_name]
            parameter_assignments = dict()
            fixed_parameters_this_action = dict()
            for param_idx, param in enumerate(action_description["params"]):
                parameter_assignments[param[0]] = plan_item_list[2 + param_idx]
            for effect in action_description["effects"]:
                for goal in self.knowledge_base.goals:
                    if goal[0] == effect[0] and goal[1] == effect[1]:
                        goal_equals_effect = True
                        for goal_param_idx, goal_param in enumerate(goal[2]):
                            if (
                                goal_param
                                != parameter_assignments[effect[2][goal_param_idx]]
                            ):
                                goal_equals_effect = False
                                break
                        if goal_equals_effect:
                            for effect_param_idx, effect_param in enumerate(effect[2]):
                                if effect_param in fixed_parameters_this_action:
                                    assert (
                                        fixed_parameters_this_action[effect_param]
                                        == goal[2][effect_param_idx]
                                    )
                                else:
                                    fixed_parameters_this_action[effect_param] = goal[
                                        2
                                    ][effect_param_idx]
            fixed_parameters_full[plan_idx] = fixed_parameters_this_action

        # Determine which actions are goal relevant and remove the rest
        fixed_parameters = list()
        sequence = list()
        for plan_idx, plan_item in enumerate(plan):
            plan_item_list = plan_item.split(" ")
            if len(fixed_parameters_full[plan_idx]) > 0:
                fixed_parameters.append(fixed_parameters_full[plan_idx])
                sequence.append(plan_item_list[1])
                break
                # TODO this break can be removed once the algorithm is adapted to computing necessary actions between
                # two actions in the sequence.

        found_plan = False
        self.knowledge_base.clear_temp()
        for _ in range(max_sample_repetitions):
            found_plan = self._sampling_loops(
                sequences_tried, given_seq=sequence, given_params=fixed_parameters
            )
            if found_plan:
                break
        # Restore initial state
        p.restoreState(stateId=self.current_state_id)
        return found_plan

    def _explore_goal_objects(self, relevant_objects, sequences_tried):
        # TODO adapt this to only sample from relevant objects

        found_plan = False
        self.knowledge_base.clear_temp()
        for _ in range(max_sample_repetitions):
            for seq_len in range(1, max_seq_len + 1):
                found_plan = self._sampling_loops(sequences_tried, seq_len=seq_len)
                if found_plan:
                    break
            if found_plan:
                break
        # Restore initial state
        p.restoreState(stateId=self.current_state_id)
        return found_plan

    def _explore_all(self, sequences_tried):
        found_plan = False
        self.knowledge_base.clear_temp()
        for _ in range(max_sample_repetitions):
            for seq_len in range(1, max_seq_len + 1):
                found_plan = self._sampling_loops(sequences_tried, seq_len=seq_len)
                if found_plan:
                    break
            if found_plan:
                break
        # Restore initial state
        p.restoreState(stateId=self.current_state_id)
        return found_plan

    # ----- Tools for sampling ------------------------------------

    def _sampling_loops(
        self, sequences_tried, given_seq=None, given_params=None, seq_len=None
    ):
        found_plan = False
        for sample_idx in range(max_samples_per_seq_len):
            # Sample sequences until an abstractly feasible one was found
            (
                success,
                sequence,
                parameter_samples,
                sequence_preconds,
            ) = self._sample_feasible_sequence(
                sequences_tried,
                given_seq=given_seq,
                given_params=given_params,
                sequence_length=seq_len,
            )
            if not success:
                print("Sampling failed. Abort searching in this sequence length.")
                break
            # count_seq_found[seq_len - 1] += 1

            # Found a feasible action sequence. Now test it.
            preplan_success = self._fulfill_preconditions(sequence_preconds)
            if not preplan_success:
                continue
            print("Preplan SUCCESS")
            # count_preplan_run_success[seq_len - 1] += 1

            # Try actual plan
            plan_success = self._execute_sampled_sequence(sequence, parameter_samples)
            if not plan_success:
                continue
            print("Sequence SUCCESS")
            # count_seq_run_success[seq_len - 1] += 1

            # Check if the goal was reached
            success = self.knowledge_base.test_goals()
            if not success:
                continue
            print("GOAL REACHED!!!")
            # count_goal_reached[seq_len - 1] += 1

            if given_seq is not None and given_params is not None:
                # Generalize action
                self.pddl_extender.generalize_action(sequence[0], parameter_samples[0])
            else:
                # Save the successful sequence and parameters.
                self.pddl_extender.create_new_action(
                    goals=self.knowledge_base.goals,
                    sequence=sequence,
                    parameters=parameter_samples,
                    sequence_preconds=sequence_preconds,
                )

            self.knowledge_base.clear_temp()
            found_plan = True
            break

        return found_plan

    def _sample_feasible_sequence(
        self, sequences_tried, sequence_length=None, given_seq=None, given_params=None
    ):
        # Sample sequences until an abstractly feasible one was found
        if given_seq is None:
            assert sequence_length is not None
            flag_sample_sequences = True
        else:
            assert given_params is not None
            flag_sample_sequences = False
            seq = given_seq

        failed_samples = 0
        success = True
        sequence_preconds = None
        while True:
            failed_samples += 1
            if failed_samples > max_failed_samples:
                success = False
                break

            if flag_sample_sequences:
                seq = self._sample_sequence(sequence_length)
            try:
                params, params_tuple = self._sample_parameters(seq, given_params)
            except NameError:
                continue
            if (tuple(seq), tuple(params_tuple),) in sequences_tried:
                continue
            sequences_tried.add((tuple(seq), tuple(params_tuple)))
            sequence_preconds = determine_sequence_preconds(
                self.knowledge_base, seq, params
            )
            if test_abstract_feasibility(
                self.knowledge_base, seq, params, sequence_preconds
            ):
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

    def _sample_parameters(self, sequence, given_params=None):
        parameter_samples = [None] * len(sequence)
        parameter_samples_tuples = [None] * len(sequence)

        # Create list of relevant items in the scene
        # TODO For now this is just adding all objects in the scene. Instead, just add objects
        # currently in proximity to the robot and the objects of interest.
        objects_of_interest_dict = self.knowledge_base.objects
        types_by_parent = invert_dict(self.knowledge_base.types)
        objects_by_type = invert_dict(objects_of_interest_dict)

        for idx_action, action in enumerate(sequence):
            parameter_samples[idx_action] = dict()
            parameters_current_action = list()
            for parameter in self.knowledge_base.actions[action]["params"]:
                obj_type = parameter[1]
                obj_name = parameter[0]

                if given_params is not None and obj_name in given_params[idx_action]:
                    # Copy the given parameter
                    obj_sample = given_params[idx_action][obj_name]
                else:
                    # Sample a value for this parameter
                    if self.knowledge_base.type_x_child_of_y(obj_type, "position"):
                        position = self._sample_position()
                        obj_sample = self.knowledge_base.add_temp_object(
                            object_type=obj_type, object_value=position
                        )
                    else:
                        objects_to_sample_from = self.knowledge_base.get_objects_by_type(
                            obj_type,
                            types_by_parent,
                            objects_by_type,
                            visible_only=True,
                        )
                        if len(objects_to_sample_from) == 0:
                            # No object of the desired type exists, sample new sequence
                            raise NameError(
                                "No object of desired type among objects of interest"
                            )
                        obj_sample = np.random.choice(list(objects_to_sample_from))
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

    # ----- Other tools ------------------------------------

    def _fulfill_preconditions(self, sequence_preconds):
        # Set up planning problem that takes us to state where all preconditions are met
        self.knowledge_base.set_temp_goals(sequence_preconds)
        plan = self.knowledge_base.solve_temp()

        if plan is False:
            return False
        else:
            # Restore initial state
            p.restoreState(stateId=self.current_state_id)

            # Execute plan to get to start of sequence
            success = self._execute_plan(plan)
            return success

    def _execute_sampled_sequence(self, sequence, parameter_samples):
        sequence_plan = list()
        for idx_action, action in enumerate(sequence):
            act_string = str(idx_action) + ": " + action
            for parameter in self.knowledge_base.actions[action]["params"]:
                act_string += " " + parameter_samples[idx_action][parameter[0]]
            sequence_plan.append(act_string)

        print("----------")
        print("Sequence: ")
        for act in sequence_plan:
            print(act)

        success = self._execute_plan(sequence_plan)
        return success

    def _execute_plan(self, plan):
        es = SequentialExecution(self.skill_set, plan, self.knowledge_base)
        es.setup()
        while True:
            success, plan_finished = es.step()
            # TODO if we run into a failure, check why this failure happened and adapt the PDDL if necessary
            if plan_finished or not success:
                break
        return success

    # def _get_objects_of_interest(self):
    #     # TODO finish and use this function
    #     interest_locations = list()

    #     # Get robot position
    #     temp = p.getBasePositionAndOrientation(self.robot_uid_)
    #     robot_pos = np.array(temp[0])
    #     interest_locations.append(robot_pos)

    #     # Get positions of objects that are in the goal description
    #     for goal in self.knowledge_base.goals:
    #         for arg in goal[2]:
    #             if arg in self.scene_objects:
    #                 pos = p.getBasePositionAndOrientation(
    #                     self.scene_objects[arg].model.uid
    #                 )
    #                 interest_locations.append(np.array(pos))
    #             elif arg in self.knowledge_lookups["position"].data:
    #                 interest_locations.append(
    #                     self.knowledge_lookups["position"].get(arg)
    #                 )

    #     # Add scene objects that are close to interest locations

    def _get_items_goal(self, objects_only=False):
        """
        Get objects that involved in the goal description
        """
        item_list = list()
        for goal in self.knowledge_base.goals:
            for arg in goal[2]:
                if objects_only:
                    if arg in self.scene_objects:
                        item_list.append(arg)
                else:
                    item_list.append(arg)
        return item_list
