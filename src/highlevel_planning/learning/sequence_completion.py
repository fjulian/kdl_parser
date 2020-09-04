from highlevel_planning.learning import logic_tools
from copy import deepcopy


def complete_sequence(sequence, parameters, relevant_objects, explorer):
    completed_sequence, completed_parameters = list(), list()
    precondition_sequence, precondition_params = list(), list()
    key_action_indices = [0] * len(sequence)

    # Determine initial state
    states = [
        ("at", "origin", "robot1"),
        ("in-reach", "origin", "robot1"),
        ("empty-hand", "robot1"),
    ]
    # TODO determine the initial state automatically

    for action_idx, action_name in enumerate(sequence):
        action_description = explorer.knowledge_base.actions[action_name]
        goals = action_description["preconds"]
        parameterized_goals = logic_tools.parametrize_predicate_list(
            goals, parameters[action_idx]
        )

        # Find sequence that makes this action possible
        plan = explorer.knowledge_base.solve_temp(
            parameterized_goals, initial_predicates=states
        )
        if plan is False:
            return False
        fill_sequence, fill_parameters = plan

        # Resample positions because the planner just randomly picked some
        resample_positions(
            fill_sequence,
            fill_parameters,
            relevant_objects,
            states,
            parameterized_goals,
            explorer,
        )

        fill_sequence_effects = logic_tools.determine_sequence_effects(
            explorer.knowledge_base, fill_sequence, fill_parameters
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
        key_action_indices[action_idx] = len(completed_sequence) - 1
    return (
        completed_sequence,
        completed_parameters,
        precondition_sequence,
        precondition_params,
        key_action_indices,
    )


def resample_positions(
    sequence, parameters, relevant_objects, initial_state, goal_state, explorer
):
    states = [(st[0], True, list(st[1:])) for st in initial_state]
    state_attribution = [(-1, -1)] * len(states)
    parameters_fixed = [] * len(sequence)
    action_descriptions = list()
    for idx, action in enumerate(sequence):
        action_descriptions.append(explorer.knowledge_base.actions[action])
        parameters_fixed.append(
            {param_spec[0]: False for param_spec in action_descriptions[idx]["params"]}
        )
        parameterized_preconds = logic_tools.parametrize_predicate_list(
            action_descriptions[idx]["preconds"], parameters[idx]
        )
        for precond_idx, precond in enumerate(parameterized_preconds):
            for state_idx, predicate in enumerate(states):
                if precond[:2] == predicate[:2] and tuple(precond[2]) == tuple(
                    predicate[2]
                ):
                    # Fix precondition parameters for current and for causing action
                    causing_action_idx = state_attribution[state_idx]
                    for precond_param_idx in range(len(precond[2])):
                        param_name = action_descriptions[idx]["preconds"][precond_idx][
                            2
                        ][precond_param_idx]
                        parameters_fixed[idx][param_name] = True

                        if causing_action_idx[0] > -1:
                            param_name = action_descriptions[causing_action_idx[0]][
                                "effects"
                            ][causing_action_idx[1]][2][precond_param_idx]
                            parameters_fixed[causing_action_idx[0]][param_name] = True

        # Apply the action
        parameterized_effects = logic_tools.parametrize_predicate_list(
            action_descriptions[idx]["effects"], parameters[idx]
        )
        state_idx_to_remove = list()
        for effect in parameterized_effects:
            for state_idx, state in enumerate(states):
                if effect[0] == state[0] and tuple(effect[2]) == tuple(state[2]):
                    state_idx_to_remove.append(state_idx)
        state_idx_to_remove.sort(reverse=True)
        for state_idx in state_idx_to_remove:
            del states[state_idx]
            del state_attribution[state_idx]
        for effect_idx, effect in enumerate(parameterized_effects):
            states.append(effect)
            state_attribution.append((idx, effect_idx))

    # Fix the parameters that achieve the goal
    for goal in goal_state:
        for state_idx, predicate in enumerate(states):
            if goal[:2] == predicate[:2] and tuple(goal[2]) == tuple(predicate[2]):
                causing_action_idx = state_attribution[state_idx]
                if causing_action_idx[0] > -1:
                    for goal_param_idx in range(len(goal[2])):
                        param_name = action_descriptions[causing_action_idx[0]][
                            "effects"
                        ][causing_action_idx[1]][2][goal_param_idx]
                        parameters_fixed[causing_action_idx[0]][param_name] = True

    # Resample position parameters that are not fixed
    for idx, action in enumerate(sequence):
        action_description = explorer.knowledge_base.actions[action]
        for param_name, param_type in action_description["params"]:
            if not parameters_fixed[idx][
                param_name
            ] and explorer.knowledge_base.is_type(
                parameters[idx][param_name], "position"
            ):
                new_position = explorer.sample_position(relevant_objects)
                new_param_value = explorer.knowledge_base.add_temp_object(
                    object_type=param_type, object_value=new_position
                )
                parameters[param_name] = new_param_value

                # TODO make sure that this temp object is made permanent if the sequence is kept
