# Imports
import numpy as np
import pybullet as p
from collections import OrderedDict
from copy import deepcopy

from highlevel_planning.execution.es_sequential_execution import (
    execute_plan_sequentially,
)
from highlevel_planning.tools.util import get_combined_aabb
from highlevel_planning.learning import logic_tools
from highlevel_planning.learning.sequence_completion import complete_sequence
from highlevel_planning.learning.precondition_discovery import precondition_discovery
from highlevel_planning.learning.exploration_tools import get_items_closeby


class Explorer:
    def __init__(
        self, skill_set, robot, scene_objects, pddl_extender, knowledge_base, config
    ):
        self.config_params = config.getparam(["explorer"])

        self.action_list = [
            act
            for act in knowledge_base.actions
            if act not in knowledge_base.meta_actions
        ]
        for rm_action in self.config_params["action_denylist"]:
            self.action_list.remove(rm_action)

        self.skill_set = skill_set
        self.robot = robot
        self.robot_uid_ = robot.model.uid
        self.scene_objects = scene_objects
        self.pddl_extender = pddl_extender
        self.knowledge_base = knowledge_base

        self.current_state_id = None
        self.metrics = None
        self.metrics_prefix = ""

    def set_metrics_prefix(self, prefix: str):
        self.metrics_prefix = prefix

    def add_metric(self, key: str, value):
        self.metrics[f"{self.metrics_prefix}_{key}"] = value

    def exploration(
        self,
        planning_failed: bool,
        demo_sequence=None,
        demo_parameters=None,
        state_id=None,
    ):
        self.metrics = OrderedDict()

        np.random.seed(0)
        sequences_tried = set()

        # Save the state the robot is currently in
        if state_id is None:
            self.current_state_id = p.saveState()
        else:
            self.current_state_id = state_id
            p.restoreState(state_id)

        # Identify objects that are involved in reaching the goal
        goal_objects = self._get_items_goal()
        radii = self.config_params["radii"]

        res = False
        if demo_sequence is not None and demo_parameters is not None:
            self.set_metrics_prefix("01_demo")
            res = self._explore_demonstration(
                demo_sequence, demo_parameters, goal_objects, sequences_tried
            )
        if not planning_failed and not res:
            self.set_metrics_prefix("02_prepend")
            closeby_objects = get_items_closeby(
                goal_objects,
                self.scene_objects,
                self.robot_uid_,
                distance_limit=0.5,  # TODO move magic number to parameters
            )
            res = self._explore_prepending_sequence(
                closeby_objects + goal_objects, sequences_tried
            )
        if not res:
            self.set_metrics_prefix("03_generalize")
            res = self._explore_generalized_action(goal_objects, sequences_tried)
        for radius in radii:
            if not res:
                self.set_metrics_prefix(f"04_rad{radius}")
                closeby_objects = get_items_closeby(
                    goal_objects,
                    self.scene_objects,
                    self.robot_uid_,
                    distance_limit=radius,
                )
                self.add_metric("closeby_objects", closeby_objects)
                res = self._explore_goal_objects(
                    sequences_tried, goal_objects + closeby_objects
                )
        return res, self.metrics

    # ----- Different sampling strategies ------------------------------------

    @staticmethod
    def get_sampling_counters_dict():
        return OrderedDict.fromkeys(
            ("valid_sequences", "preplan_success", "plan_success", "goal_reached"), 0
        )

    def _explore_demonstration(
        self, demo_sequence, demo_parameters, relevant_objects, sequences_tried
    ):
        print("Exploring demonstration ...")
        found_plan = self._sampling_loops_caller(
            relevant_objects,
            len(demo_sequence),
            len(demo_sequence),
            sequences_tried,
            demo_sequence,
            demo_parameters,
        )
        return found_plan

    def _explore_prepending_sequence(self, relevant_objects, sequences_tried):
        print("Exploring prepending sequence ...")
        plan = self.knowledge_base.solve()
        if not plan:
            return False
        sequence, parameters = plan
        relevant_sequence, relevant_parameters = self._extract_goal_relevant_sequence(
            sequence, parameters, fix_all_params=True
        )

        min_sequence_length = len(relevant_sequence) + 1
        max_sequence_length = np.max(
            (self.config_params["max_sequence_length"], len(relevant_sequence) + 1)
        )
        found_plan = self._sampling_loops_caller(
            relevant_objects,
            min_sequence_length,
            max_sequence_length,
            sequences_tried,
            relevant_sequence,
            relevant_parameters,
            planning_failed=False,
        )
        return found_plan

    def _explore_generalized_action(self, relevant_objects, sequences_tried):
        print("Exploring generalizing action ...")

        # Check if an action with a similar effect already exists
        self.knowledge_base.clear_temp()
        for obj in relevant_objects:
            self.knowledge_base.generalize_temp_object(obj)
        plan = self.knowledge_base.solve_temp(self.knowledge_base.goals)
        if not plan:
            return False
        sequence, parameters = plan

        relevant_sequence, relevant_parameters = self._extract_goal_relevant_sequence(
            sequence, parameters
        )

        min_sequence_length = len(relevant_sequence)
        max_sequence_length = np.max(
            (self.config_params["max_sequence_length"], len(relevant_sequence))
        )
        found_plan = self._sampling_loops_caller(
            relevant_objects,
            min_sequence_length,
            max_sequence_length,
            sequences_tried,
            relevant_sequence,
            relevant_parameters,
        )
        return found_plan

    def _explore_goal_objects(self, sequences_tried, relevant_objects=None):
        print("Exploring goal objects ...")
        min_sequence_length = 1
        max_sequence_length = self.config_params["max_sequence_length"]
        found_plan = self._sampling_loops_caller(
            relevant_objects, min_sequence_length, max_sequence_length, sequences_tried
        )
        return found_plan

    # ----- Tools for sampling ------------------------------------

    def _sampling_loops_caller(
        self,
        relevant_objects,
        min_sequence_length,
        max_sequence_length,
        sequences_tried,
        given_sequence=None,
        given_parameters=None,
        planning_failed=True,
    ):
        found_plan = False
        self.knowledge_base.clear_temp()
        sampling_counters = self.get_sampling_counters_dict()
        for rep_idx in range(self.config_params["max_sample_repetitions"]):
            print(f"Repitition {rep_idx}")
            for seq_len in range(min_sequence_length, max_sequence_length + 1):
                print(f"Sequence length {seq_len}")
                found_plan = self._sampling_loops(
                    sequences_tried,
                    sampling_counters,
                    seq_len,
                    given_seq=given_sequence,
                    given_params=given_parameters,
                    relevant_objects=relevant_objects,  # TODO check if it makes sense that we only sample goal actions
                    planning_failed=planning_failed,
                )
                if found_plan:
                    self.add_metric("successful_seq_len", seq_len)
                    break
            if found_plan:
                break

        # Store counters
        for counter in sampling_counters:
            self.add_metric(counter, sampling_counters[counter])
        self.add_metric("found_plan", found_plan)

        # Restore initial state
        p.restoreState(stateId=self.current_state_id)
        return found_plan

    def _sampling_loops(
        self,
        sequences_tried,
        counters,
        seq_len: int,
        given_seq=None,
        given_params=None,
        relevant_objects=None,
        do_complete_sequence=False,
        planning_failed=True,
    ):
        found_plan = False
        for sample_idx in range(self.config_params["max_samples_per_sequence_length"]):
            # Restore initial state
            p.restoreState(stateId=self.current_state_id)

            # Sample sequences until an abstractly feasible one was found
            (success, completion_result) = self._sample_feasible_sequence(
                sequences_tried,
                seq_len,
                given_seq=given_seq,
                given_params=given_params,
                relevant_objects=relevant_objects,
                do_complete_sequence=do_complete_sequence,
            )
            if not success:
                print("Sampling failed. Abort searching in this sequence length.")
                break
            counters["valid_sequences"] += 1

            test_success = self._test_completed_sequence(completion_result)
            counters["preplan_success"] += test_success[0]
            counters["plan_success"] += test_success[1]
            counters["goal_reached"] += test_success[2]
            if test_success[2] == 0:
                continue
            print("SUCCESS. Achieved goal, now extending symbolic description.")

            # -----------------------------------------------
            # Extend the symbolic description appropriately

            completed_sequence = completion_result[0]
            completed_parameters = completion_result[1]
            if len(completed_sequence) == 1:
                if given_seq is not None and given_params is not None:
                    # Generalize action
                    assert (
                        len(completed_sequence) == 1
                    ), "If the given sequence is longer than 1, this code cannot deal with it yet"
                    self.pddl_extender.generalize_action(
                        completed_sequence[0], completed_parameters[0]
                    )
                else:
                    # Save the successful sequence and parameters.
                    self.pddl_extender.create_new_action(
                        goals=self.knowledge_base.goals,
                        meta_preconditions=None,
                        sequence=completed_sequence,
                        parameters=completed_parameters,
                    )
            else:
                key_actions = completion_result[4]
                if len(key_actions) > 2:
                    raise NotImplementedError

                # Try to find actual key actions
                i = 1
                last_working_completion_result = completion_result
                while True:
                    # Idea for more than 2 key actions: shift foremost back if it is at least two before next one.
                    # Otherwise shift next one back according to the same rule. If next one is the last one, eliminate
                    # next one.
                    # TODO implement this.
                    if key_actions[0] + i > key_actions[1]:
                        break
                    elif key_actions[0] + i < key_actions[1]:
                        modified_sequence = [
                            completed_sequence[key_actions[0] + i],
                            completed_sequence[key_actions[1]],
                        ]
                        modified_parameters = [
                            completed_parameters[key_actions[0] + i],
                            completed_parameters[key_actions[1]],
                        ]
                    else:
                        modified_sequence = [completed_sequence[key_actions[1]]]
                        modified_parameters = [completed_parameters[key_actions[1]]]
                    modified_completion_result = complete_sequence(
                        modified_sequence, modified_parameters, relevant_objects, self
                    )
                    if not modified_completion_result:
                        i += 1
                        continue
                    test_success = self._test_completed_sequence(
                        modified_completion_result
                    )
                    if not test_success[2]:
                        i += 1
                        continue
                    last_working_completion_result = modified_completion_result
                    i += 1

                completed_sequence = last_working_completion_result[0]
                completed_parameters = last_working_completion_result[1]
                key_actions = last_working_completion_result[4]

                effects_last_action = list()
                if len(last_working_completion_result[0]) > 1:
                    # Precondition discovery
                    precondition_candidates, precondition_actions = precondition_discovery(
                        relevant_objects, last_working_completion_result, self
                    )
                    precondition_idx = 0
                    for key_action_idx in range(len(key_actions) - 1):
                        effects_this_action = list()
                        while True:
                            if (
                                precondition_idx >= len(precondition_actions)
                                or precondition_actions[precondition_idx]
                                > key_actions[key_action_idx]
                            ):
                                break
                            effects_this_action.append(
                                precondition_candidates[precondition_idx]
                            )
                            precondition_idx += 1
                        self.pddl_extender.create_new_action(
                            goals=effects_this_action,
                            meta_preconditions=effects_last_action,
                            sequence=[completed_sequence[key_actions[key_action_idx]]],
                            parameters=[
                                completed_parameters[key_actions[key_action_idx]]
                            ],
                        )
                        effects_last_action = deepcopy(effects_this_action)

                # Add action that reaches the goal
                if planning_failed:
                    self.pddl_extender.create_new_action(
                        goals=self.knowledge_base.goals,
                        meta_preconditions=effects_last_action,
                        sequence=[completed_sequence[key_actions[-1]]],
                        parameters=[completed_parameters[key_actions[-1]]],
                    )
                else:
                    self.pddl_extender.generalize_action(
                        action_name=completed_sequence[key_actions[-1]],
                        parameters=completed_parameters[key_actions[-1]],
                        additional_preconditions=effects_last_action,
                    )

            found_plan = True
            break

        return found_plan

    def _sample_feasible_sequence(
        self,
        sequences_tried: set,
        sequence_length: int,
        given_seq: list = None,
        given_params: list = None,
        relevant_objects=None,
        do_complete_sequence: bool = False,
    ):
        """
        Sample sequences until an abstractly feasible one was found
        """

        if given_seq is None:
            given_seq = list()
            given_params = list()
        else:
            assert len(given_seq) <= sequence_length
            given_seq = deepcopy(given_seq)
            given_params = deepcopy(given_params)

        failed_samples = 0
        success = True
        completion_result = None
        while True:
            failed_samples += 1
            if failed_samples > self.config_params["max_failed_samples"]:
                success = False
                break

            if len(given_seq) < sequence_length:
                pre_seq = self._sample_sequence(sequence_length - len(given_seq))
                seq = pre_seq + given_seq
                pre_params = [{}] * (sequence_length - len(given_seq))
                fixed_params = pre_params + given_params
            else:
                seq = given_seq
                fixed_params = given_params

            try:
                params, params_tuple = self._sample_parameters(
                    seq, fixed_params, relevant_objects
                )
            except NameError:
                continue
            sequence_tuple = (tuple(seq), tuple(params_tuple))
            if sequence_tuple in sequences_tried:
                continue
            sequences_tried.add(sequence_tuple)

            # if given_seq is None or do_complete_sequence:
            # Fill in the gaps of the sequence to make it feasible
            completion_result = complete_sequence(seq, params, relevant_objects, self)
            if completion_result is False:
                continue
            # else:
            #     # TODO test if this is needed or if we can run this through the completion anyways
            #     completed_sequence = seq
            #     completed_parameters = params
            #     sequence_preconds = logic_tools.determine_sequence_preconds(
            #         self.knowledge_base, completed_sequence, completed_parameters
            #     )
            #     precondition_plan = self.knowledge_base.solve_temp(sequence_preconds)
            #     if not precondition_plan:
            #         success = False
            #         break
            #     precondition_sequence, precondition_parameters = precondition_plan
            break
        return success, completion_result

    def _sample_sequence(self, length, no_action_repetition=False):
        """
        Function to sample actions of a sequence.

        :param length [int]: length of the sequence to be sampled
        :param no_action_repetition [bool]: If true, there will not be twice the same action in a row.
        :return:
        """
        # Generate the sequence
        sequence = list()
        for _ in range(length):
            if no_action_repetition:
                while True:
                    temp = np.random.choice(self.action_list)
                    if len(sequence) == 0:
                        sequence.append(temp)
                        break
                    elif temp != sequence[-1]:
                        sequence.append(temp)
                        break
            else:
                sequence.append(np.random.choice(self.action_list))
        return sequence

    def _sample_parameters(self, sequence, given_params=None, relevant_objects=None):
        parameter_samples = list()
        parameter_samples_tuples = list()

        # Create list of relevant items in the scene
        objects_of_interest_dict = dict()
        for obj in relevant_objects:
            objects_of_interest_dict[obj] = self.knowledge_base.objects[obj]
        objects_of_interest_dict["robot1"] = self.knowledge_base.objects["robot1"]
        types_by_parent = logic_tools.invert_dict(self.knowledge_base.types)
        objects_of_interest_by_type = logic_tools.invert_dict(objects_of_interest_dict)

        for idx_action, action in enumerate(sequence):
            parameter_samples.append(dict())
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
                        position = self.sample_position(relevant_objects)
                        obj_sample = self.knowledge_base.add_temp_object(
                            object_type=obj_type, object_value=position
                        )
                    else:
                        objects_to_sample_from = self.knowledge_base.get_objects_by_type(
                            obj_type,
                            types_by_parent,
                            objects_of_interest_by_type,
                            visible_only=True,
                        )
                        if len(objects_to_sample_from) == 0:
                            # If no suitable object is in the objects of interest, check among all objects
                            objects_all_by_type = logic_tools.invert_dict(
                                self.knowledge_base.objects
                            )
                            objects_to_sample_from = self.knowledge_base.get_objects_by_type(
                                obj_type,
                                types_by_parent,
                                objects_all_by_type,
                                visible_only=False,
                            )
                            if len(objects_to_sample_from) == 0:
                                # No object of the desired type exists, sample new sequence
                                raise NameError(
                                    "No object of desired type among objects of interest"
                                )
                        obj_sample = np.random.choice(list(objects_to_sample_from))
                parameter_samples[idx_action][obj_name] = obj_sample
                parameters_current_action.append(obj_sample)
            parameter_samples_tuples.append(tuple(parameters_current_action))
        assert len(parameter_samples) == len(sequence)
        assert len(parameter_samples_tuples) == len(sequence)
        return parameter_samples, parameter_samples_tuples

    def sample_position(self, relevant_objects):
        # Choose one goal object next to which to sample
        obj_sample = np.random.choice(relevant_objects)
        uid = self.scene_objects[obj_sample].model.uid

        # Get robot base position
        arm_base_pos, _ = self.robot.get_link_pose("panda_link0")

        # Get AABB
        bounding_box = get_combined_aabb(uid)

        # Inflate the bounding box
        min_coords = bounding_box[0]
        max_coords = bounding_box[1]
        max_coords += self.config_params["bounding_box_inflation_length"]
        min_coords -= self.config_params["bounding_box_inflation_length"]
        min_coords[2] = np.max([min_coords[2], arm_base_pos[2] - 0.1])

        assert min_coords[2] < max_coords[2]

        # Sample
        sample = np.random.uniform(low=min_coords, high=max_coords)
        return sample

    # ----- Other tools ------------------------------------

    def _extract_goal_relevant_sequence(
        self, sequence, parameters, fix_all_params: bool = False
    ):
        """

        Args:
            sequence:
            parameters:
            fix_all_params: If set to true, all parameters of an action that contributes to the goal are fix.
                            If not, only the goal relevant parameters are fixed.

        Returns:

        """
        # Extract parameters from plan
        fixed_parameters_full = list()
        for action_idx, action_name in enumerate(sequence):
            action_description = self.knowledge_base.actions[action_name]
            parameter_assignments = parameters[action_idx]
            fixed_parameters_this_action = dict()
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
            if fix_all_params and len(fixed_parameters_this_action) > 0:
                for param in action_description["params"]:
                    if param[0] not in fixed_parameters_this_action:
                        fixed_parameters_this_action[param[0]] = parameter_assignments[
                            param[0]
                        ]
            fixed_parameters_full.append(fixed_parameters_this_action)
        assert len(fixed_parameters_full) == len(sequence)

        # Determine which actions are goal relevant and remove the rest
        relevant_parameters = list()
        relevant_sequence = list()
        for action_idx, action_name in enumerate(sequence):
            if len(fixed_parameters_full[action_idx]) > 0:
                relevant_parameters.append(fixed_parameters_full[action_idx])
                relevant_sequence.append(action_name)
        return relevant_sequence, relevant_parameters

    def _get_items_goal(self, objects_only=False):
        """
        Get objects that are involved in the goal description
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

    def _test_completed_sequence(self, completion_result: dict):
        # Restore initial state
        p.restoreState(stateId=self.current_state_id)

        success = np.array([0, 0, 0])
        (
            completed_sequence,
            completed_parameters,
            precondition_sequence,
            precondition_parameters,
            key_actions,
        ) = completion_result

        # Found a feasible action sequence. Now test it.
        preplan_success = execute_plan_sequentially(
            precondition_sequence,
            precondition_parameters,
            self.skill_set,
            self.knowledge_base,
        )
        if not preplan_success:
            return success
        success[0] = 1

        # Try actual plan
        plan_success = execute_plan_sequentially(
            completed_sequence,
            completed_parameters,
            self.skill_set,
            self.knowledge_base,
        )
        if not plan_success:
            return success
        success[1] = 1

        # Check if the goal was reached
        goal_success = self.knowledge_base.test_goals()
        if not goal_success:
            return success
        success[2] = 1
        return success
