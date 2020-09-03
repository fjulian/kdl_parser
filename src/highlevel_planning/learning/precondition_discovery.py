import numpy as np
import pybullet as p
from copy import deepcopy
from itertools import product


def precondition_discovery(relevant_objects, completion_results, explorer):
    precondition_candidates = list()

    (
        completed_sequence,
        completed_parameters,
        precondition_sequence,
        precondition_params,
        key_action_indices,
    ) = completion_results

    # Restore initial state
    p.restoreState(stateId=explorer.current_state_id)

    relevant_predicates = determine_relevant_predicates(
        relevant_objects, explorer.knowledge_base
    )

    # Check the predicates
    pre_predicates = measure_predicates(relevant_predicates, explorer.knowledge_base)

    # Execute the pre-condition sequence
    res = explorer._execute_plan(precondition_sequence, precondition_params)
    if not res:
        return False

    current_predicates = measure_predicates(
        relevant_predicates, explorer.knowledge_base
    )
    new_side_effects = detect_predicate_changes(
        relevant_predicates,
        pre_predicates,
        current_predicates,
        precondition_sequence,
        precondition_params,
        explorer,
    )
    precondition_candidates.extend(new_side_effects)

    # Execute actions one by one, check for non-effect predicate changes
    for idx, action in enumerate(completed_sequence):
        pre_predicates = deepcopy(current_predicates)
        res = explorer._execute_plan([action], [completed_parameters[idx]])
        if not res:
            return False
        current_predicates = measure_predicates(
            relevant_predicates, explorer.knowledge_base
        )
        new_side_effects = detect_predicate_changes(
            relevant_predicates,
            pre_predicates,
            current_predicates,
            [action],
            [completed_parameters[idx]],
            explorer,
        )
        precondition_candidates.extend(new_side_effects)
    return precondition_candidates


def determine_relevant_predicates(relevant_objects, knowledge_base):
    """
    Determine all predicates of objects involved in this action and objects that are close to them
    """
    predicate_descriptions = knowledge_base.predicate_funcs.descriptions
    relevant_predicates = list()
    for pred in predicate_descriptions:
        parameters = predicate_descriptions[pred]

        # Find possible parameter assignments
        parameter_assignments = list()
        for param_idx, param in enumerate(parameters):
            assignments_this_param = list()
            if param[1] == "robot":
                assignments_this_param.append("robot1")
            else:
                for obj in relevant_objects:
                    if knowledge_base.is_type(obj, param[1]):
                        assignments_this_param.append(obj)
            parameter_assignments.append(assignments_this_param)

        for parametrization in product(*parameter_assignments):
            relevant_predicates.append((pred, parametrization))
    return relevant_predicates


def measure_predicates(predicates, knowledge_base):
    measurements = list()
    for pred in predicates:
        res = knowledge_base.predicate_funcs.call[pred[0]](*pred[1])
        measurements.append(res)
    return measurements


def detect_predicate_changes(
    predicate_definitions,
    old_predicates,
    new_predicates,
    action_sequence,
    action_parameters,
    explorer,
):
    side_effects = list()

    changed_indices = np.nonzero(np.logical_xor(old_predicates, new_predicates))
    assert len(changed_indices) == 1
    changed_indices = changed_indices[0]
    for idx in changed_indices:
        predicate_def = predicate_definitions[idx]
        if (
            predicate_def[0]
            not in explorer.config_params["predicate_precondition_allowlist"]
        ):
            continue
        predicate_state = new_predicates[idx]

        # Check if the last action(s) have this predicate change in their effect list. If yes, ignore.
        predicate_expected = False
        for action_idx, action in enumerate(action_sequence):
            action_descr = explorer.knowledge_base.actions[action]
            for effect in action_descr["effects"]:
                if (
                    effect[0] == predicate_def[0]
                    and effect[1] == predicate_state
                    and effect[2] == action_parameters[action_idx]
                ):
                    predicate_expected = True
                    break
            if predicate_expected:
                break
        if predicate_expected:
            continue

        # If we reach here, this is a candidate for the precondition we are trying to determine.
        side_effects.append(predicate_def)
    return side_effects
