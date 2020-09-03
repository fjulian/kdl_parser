from highlevel_planning.learning import logic_tools
from copy import deepcopy


def complete_sequence(sequence, parameters, knowledge_base):
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
        action_description = knowledge_base.actions[action_name]
        goals = action_description["preconds"]
        parameterized_goals = logic_tools.parametrize_predicate_list(
            goals, parameters[action_idx]
        )

        # Find sequence that makes this action possible
        plan = knowledge_base.solve_temp(parameterized_goals, initial_predicates=states)
        if plan is False:
            return False

        # Parse sequence
        fill_sequence, fill_parameters = plan
        fill_sequence_effects = logic_tools.determine_sequence_effects(
            knowledge_base, fill_sequence, fill_parameters
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
