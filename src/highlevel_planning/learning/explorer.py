# Imports
import numpy as np
from copy import deepcopy
import pybullet as p

from highlevel_planning.execution.es_sequential_execution import SequentialExecution
from highlevel_planning.tools.util import get_combined_aabb
from highlevel_planning.learning import logic_tools


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
        self.robot_uid_ = robot._model.uid
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
        goal_objects = self._get_items_goal()
        radii = [0.1, 1.5, 10.0]

        res = self._explore_generalized_action(goal_objects, sequences_tried)
        for radius in radii:
            if not res:
                closeby_objects = self._get_items_closeby(goal_objects, radius=radius)
                res = self._explore_goal_objects(
                    sequences_tried, goal_objects + closeby_objects
                )
        return res

    # ----- Different sampling strategies ------------------------------------

    def _explore_generalized_action(self, relevant_objects, sequences_tried):

        # Check if an action with a similar effect already exists
        self.knowledge_base.clear_temp()
        for obj in relevant_objects:
            self.knowledge_base.generalize_temp_object(obj)
        plan = self.knowledge_base.solve_temp(self.knowledge_base.goals)
        if not plan:
            return False
        sequence, parameters = plan

        # Extract parameters from plan
        fixed_parameters_full = [None] * len(sequence)
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
            fixed_parameters_full[action_idx] = fixed_parameters_this_action

        # Determine which actions are goal relevant and remove the rest
        fixed_parameters = list()
        relevant_sequence = list()
        for action_idx, action_name in enumerate(sequence):
            if len(fixed_parameters_full[action_idx]) > 0:
                fixed_parameters.append(fixed_parameters_full[action_idx])
                relevant_sequence.append(action_name)
                break
                # TODO this break can be removed once the algorithm is adapted to computing necessary actions between
                # two actions in the sequence.

        found_plan = False
        self.knowledge_base.clear_temp()
        for _ in range(self.config_params["max_sample_repetitions"]):
            found_plan = self._sampling_loops(
                sequences_tried,
                given_seq=relevant_sequence,
                given_params=fixed_parameters,
                relevant_objects=relevant_objects,  # TODO check if it actually makes sense that we only sample goal actions here
            )
            if found_plan:
                break
        # Restore initial state
        p.restoreState(stateId=self.current_state_id)
        return found_plan

    def _explore_goal_objects(self, sequences_tried, relevant_objects=None):
        found_plan = False
        self.knowledge_base.clear_temp()
        for _ in range(self.config_params["max_sample_repetitions"]):
            for seq_len in range(1, self.config_params["max_sequence_length"] + 1):
                found_plan = self._sampling_loops(
                    sequences_tried, seq_len=seq_len, relevant_objects=relevant_objects
                )
                if found_plan:
                    break
            if found_plan:
                break
        # Restore initial state
        p.restoreState(stateId=self.current_state_id)
        return found_plan

    # ----- Tools for sampling ------------------------------------

    def _sampling_loops(
        self,
        sequences_tried,
        given_seq=None,
        given_params=None,
        seq_len=None,
        relevant_objects=None,
    ):
        found_plan = False
        for sample_idx in range(
            self.config_params["max_samples_per_seqequence_length"]
        ):
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
                given_seq=given_seq,
                given_params=given_params,
                sequence_length=seq_len,
                relevant_objects=relevant_objects,
            )
            if not success:
                print("Sampling failed. Abort searching in this sequence length.")
                break
            # count_seq_found[seq_len - 1] += 1

            # Found a feasible action sequence. Now test it.
            preplan_success = self._execute_plan(
                precondition_sequence, precondition_parameters
            )
            if not preplan_success:
                continue
            print("Preplan SUCCESS")

            # Try actual plan
            plan_success = self._execute_plan(completed_sequence, completed_parameters)
            if not plan_success:
                continue
            print("Sequence SUCCESS")

            # Check if the goal was reached
            success = self.knowledge_base.test_goals()
            if not success:
                continue
            print("GOAL REACHED!!!")

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

            self.knowledge_base.clear_temp()
            found_plan = True
            break

        return found_plan

    def _sample_feasible_sequence(
        self,
        sequences_tried,
        sequence_length=None,
        given_seq=None,
        given_params=None,
        relevant_objects=None,
    ):
        # Sample sequences until an abstractly feasible one was found
        if given_seq is None:
            assert sequence_length is not None
            flag_sample_sequences = True
            seq = None
        else:
            assert given_params is not None
            flag_sample_sequences = False
            seq = given_seq

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

            if flag_sample_sequences:
                seq = self._sample_sequence(sequence_length)
            try:
                params, params_tuple = self._sample_parameters(
                    seq, given_params, relevant_objects
                )
            except NameError:
                continue
            if (tuple(seq), tuple(params_tuple),) in sequences_tried:
                continue
            sequences_tried.add((tuple(seq), tuple(params_tuple)))

            if given_seq is None:
                # Fill in the gaps of the sequence to make it feasible
                completion_result = self.complete_sequence(seq, params)
                if completion_result is False:
                    continue
                (
                    completed_sequence,
                    completed_parameters,
                    precondition_sequence,
                    precondition_parameters,
                ) = completion_result
            else:
                completed_sequence = seq
                completed_parameters = params
                sequence_preconds = logic_tools.determine_sequence_preconds(
                    self.knowledge_base, completed_sequence, completed_parameters
                )
                self.knowledge_base.clear_temp()
                precondition_plan = self.knowledge_base.solve_temp(sequence_preconds)
                if not precondition_plan:
                    success = False
                    break
                precondition_sequence, precondition_parameters = precondition_plan
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
        parameter_samples = [None] * len(sequence)
        parameter_samples_tuples = [None] * len(sequence)

        # Create list of relevant items in the scene
        objects_of_interest_dict = dict()
        for obj in relevant_objects:
            objects_of_interest_dict[obj] = self.knowledge_base.objects[obj]
        objects_of_interest_dict["robot1"] = self.knowledge_base.objects["robot1"]
        types_by_parent = logic_tools.invert_dict(self.knowledge_base.types)
        objects_by_type = logic_tools.invert_dict(objects_of_interest_dict)

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
                        position = self._sample_position(relevant_objects)
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

    def _sample_position(self, relevant_objects):
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

    def _execute_plan(self, sequence, parameters):
        es = SequentialExecution(
            self.skill_set, sequence, parameters, self.knowledge_base
        )
        es.setup()
        while True:
            success, plan_finished, msgs = es.step()
            # TODO if we run into a failure, check why this failure happened and adapt the PDDL if necessary
            if plan_finished or not success:
                break
        return success

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

    def _get_items_closeby(self, goal_objects, radius=0.5):

        # Get robot position
        temp = p.getBasePositionAndOrientation(self.robot_uid_)
        robot_pos = np.array(temp[0])
        interest_locations = np.array([robot_pos])

        # Get positions of objects that are in the goal description
        for obj in goal_objects:
            interest_locations = np.vstack(
                (interest_locations, self._get_object_position(obj))
            )

        # Add scene objects that are close to interest locations
        closeby_objects = list()
        for obj in self.scene_objects:
            if obj in goal_objects:
                continue
            obj_pos = self._get_object_position(obj)
            distances = np.linalg.norm(interest_locations - obj_pos, axis=1)
            if np.any(distances < radius):
                closeby_objects.append(obj)
        return closeby_objects

    def _get_object_position(self, object_name):
        if object_name in self.scene_objects:
            pos, _ = p.getBasePositionAndOrientation(
                self.scene_objects[object_name].model.uid
            )
            return np.array(pos)
        elif object_name in self.knowledge_base.lookup_table:
            return self.knowledge_base.lookup_table[object_name]
        else:
            raise ValueError("Invalid object")

    def complete_sequence(self, sequence, parameters):
        completed_sequence, completed_parameters = list(), list()
        precondition_sequence, precondition_params = list(), list()

        # Determine initial state
        states = [
            ("at", "origin", "robot1"),
            ("in-reach", "origin", "robot1"),
            ("empty-hand", "robot1"),
        ]

        for action_idx, action_name in enumerate(sequence):
            action_description = self.knowledge_base.actions[action_name]
            goals = action_description["preconds"]
            parameterized_goals = logic_tools.parametrize_predicate_list(
                goals, parameters[action_idx]
            )

            # Find sequence that makes this action possible
            self.knowledge_base.clear_temp()
            plan = self.knowledge_base.solve_temp(
                parameterized_goals, initial_predicates=states
            )
            if plan is False:
                return False

            # Parse sequence
            fill_sequence, fill_parameters = plan
            fill_sequence_effects = logic_tools.determine_sequence_effects(
                self.knowledge_base, fill_sequence, fill_parameters
            )

            # Apply fill sequence to current state
            logic_tools.apply_effects_to_state(states, fill_sequence_effects)

            # Apply actual action to current state
            parameterized_effects = logic_tools.parametrize_predicate_list(
                action_description["effects"], parameters[action_idx]
            )
            logic_tools.apply_effects_to_state(states, parameterized_effects)

            # Save the sequence extension
            if action_idx == 0:
                precondition_sequence = deepcopy(fill_sequence)
                precondition_params = deepcopy(fill_parameters)
            else:
                completed_sequence.extend(fill_sequence)
                completed_parameters.extend(fill_parameters)
            completed_sequence.append(action_name)
            completed_parameters.append(parameters[action_idx])
        return (
            completed_sequence,
            completed_parameters,
            precondition_sequence,
            precondition_params,
        )
