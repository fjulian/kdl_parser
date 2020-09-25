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
            self.add_metric("result", res)
        if not planning_failed and not res:
            self.set_metrics_prefix("02_prepend")
            closeby_objects = self._get_items_closeby(
                goal_objects, distance_limit=0.5  # TODO move magic number to parameters
            )
            res = self._explore_prepending_sequence(closeby_objects, sequences_tried)
        if not res:
            self.set_metrics_prefix("02_generalize")
            res = self._explore_generalized_action(goal_objects, sequences_tried)
            self.add_metric("result", res)
        for radius in radii:
            if not res:
                self.set_metrics_prefix(f"03_rad{radius}")
                closeby_objects = self._get_items_closeby(
                    goal_objects, distance_limit=radius
                )
                self.add_metric("closeby_objects", closeby_objects)
                res = self._explore_goal_objects(
                    sequences_tried, goal_objects + closeby_objects
                )
                self.add_metric("result", res)
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
        plan = self.knowledge_base.solve()
        if not plan:
            return False
        sequence, parameters = plan
        relevant_sequence, relevant_parameters = self._extract_goal_relevant_sequence(
            sequence, parameters
        )

        min_sequence_length = len(relevant_sequence)
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
        )
        return found_plan

    def _explore_generalized_action(self, relevant_objects, sequences_tried):
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
    ):
        found_plan = False
        self.knowledge_base.clear_temp()
        sampling_counters = self.get_sampling_counters_dict()
        for _ in range(self.config_params["max_sample_repetitions"]):
            for seq_len in range(min_sequence_length, max_sequence_length + 1):
                found_plan = self._sampling_loops(
                    sequences_tried,
                    sampling_counters,
                    seq_len,
                    given_seq=given_sequence,
                    given_params=given_parameters,
                    relevant_objects=relevant_objects,  # TODO check if it makes sense that we only sample goal actions
                )
                if found_plan:
                    self.add_metric("successful_seq_len", seq_len)
                    break
            if found_plan:
                break

        # Store counters
        for counter in sampling_counters:
            self.add_metric(counter, sampling_counters[counter])

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
    ):
        found_plan = False
        for sample_idx in range(self.config_params["max_samples_per_sequence_length"]):
            # Restore initial state
            p.restoreState(stateId=self.current_state_id)

            # Sample sequences until an abstractly feasible one was found
            (
                success,
                completed_sequence,
                completed_parameters,
                precondition_sequence,
                precondition_parameters,
            ) = self._sample_feasible_sequence(
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

            # Found a feasible action sequence. Now test it.
            preplan_success = execute_plan_sequentially(
                precondition_sequence,
                precondition_parameters,
                self.skill_set,
                self.knowledge_base,
            )
            if not preplan_success:
                continue
            counters["preplan_success"] += 1

            # Try actual plan
            plan_success = execute_plan_sequentially(
                completed_sequence,
                completed_parameters,
                self.skill_set,
                self.knowledge_base,
            )
            if not plan_success:
                continue
            counters["plan_success"] += 1

            # Check if the goal was reached
            success = self.knowledge_base.test_goals()
            if not success:
                continue
            counters["goal_reached"] += 1

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
                    sequence=completed_sequence,
                    parameters=completed_parameters,
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
        completed_sequence = None
        completed_parameters = None
        precondition_sequence = None
        precondition_parameters = None
        while True:
            failed_samples += 1
            if failed_samples > self.config_params["max_failed_samples"]:
                success = False
                break

            if len(given_seq) < sequence_length:
                pre_seq = self._sample_sequence(sequence_length - len(given_seq))
                seq = pre_seq + given_seq
                pre_params = [{}] * (sequence_length - len(given_seq))
                given_params = pre_params + given_params
            else:
                seq = given_seq

            try:
                params, params_tuple = self._sample_parameters(
                    seq, given_params, relevant_objects
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
            (
                completed_sequence,
                completed_parameters,
                precondition_sequence,
                precondition_parameters,
                _,
            ) = completion_result
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
        return (
            success,
            completed_sequence,
            completed_parameters,
            precondition_sequence,
            precondition_parameters,
        )

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
        objects_by_type = logic_tools.invert_dict(objects_of_interest_dict)

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

    def _extract_goal_relevant_sequence(self, sequence, parameters):
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
            fixed_parameters_full.append(fixed_parameters_this_action)
        assert len(fixed_parameters_full) == len(sequence)

        # Determine which actions are goal relevant and remove the rest
        relevant_parameters = list()
        relevant_sequence = list()
        for action_idx, action_name in enumerate(sequence):
            if len(fixed_parameters_full[action_idx]) > 0:
                relevant_parameters.append(fixed_parameters_full[action_idx])
                relevant_sequence.append(action_name)
                break
                # TODO this break can be removed once the algorithm is adapted to computing necessary actions between
                # two actions in the sequence.
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

    def _get_items_closeby(self, goal_objects, distance_limit=0.5):
        closeby_objects = set()
        for obj in self.scene_objects:
            if obj in goal_objects:
                continue

            obj_uid = self.scene_objects[obj].model.uid
            ret = p.getClosestPoints(
                self.robot_uid_, obj_uid, distance=1.2 * distance_limit
            )
            if len(ret) > 0:
                distances = np.array([r[8] for r in ret])
                distance = np.min(distances)
                if distance <= distance_limit:
                    closeby_objects.add(obj)
                    continue

            for goal_obj in goal_objects:
                goal_obj_uid = self.scene_objects[goal_obj].model.uid
                ret = p.getClosestPoints(
                    obj_uid, goal_obj_uid, distance=1.2 * distance_limit
                )
                if len(ret) == 0:
                    continue
                distances = np.array([r[8] for r in ret])
                distance = np.min(distances)
                if distance <= distance_limit:
                    closeby_objects.add(obj)
        return list(closeby_objects)
