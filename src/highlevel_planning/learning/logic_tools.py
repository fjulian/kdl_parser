from copy import deepcopy


def parametrize_predicate(predicate, action_parameters):
    return (
        predicate[0],
        predicate[1],
        tuple([action_parameters[obj_name] for obj_name in predicate[2]]),
    )


def determine_sequence_preconds(knowledge_base, sequence, parameters):
    seq_preconds = list()
    for action_idx, action_id in reversed(list(enumerate(sequence))):
        action_descr = knowledge_base.actions[action_id]

        # Remove all effects of this action from the precond list
        preconds_to_remove = list()
        for seq_precond in seq_preconds:
            for effect in action_descr["effects"]:
                parametrized_effect = parametrize_predicate(
                    effect, parameters[action_idx]
                )
                if seq_precond == parametrized_effect:
                    preconds_to_remove.append(seq_precond)
        for seq_precond in preconds_to_remove:
            seq_preconds.remove(seq_precond)

        # Add all preconditions of this action to the precond list
        for precond in action_descr["preconds"]:
            parametrized_precond = parametrize_predicate(
                precond, parameters[action_idx]
            )
            if parametrized_precond not in seq_preconds:
                seq_preconds.append(parametrized_precond)
    return seq_preconds


def determine_sequence_effects(knowledge_base, sequence, parameters):
    seq_effects = list()
    for action_idx, action_id in enumerate(sequence):
        action_descr = knowledge_base.actions[action_id]

        # Remove colliding effects from the effect list
        effects_to_remove = list()
        for effect in action_descr["effects"]:
            parametrized_effect = parametrize_predicate(effect, parameters[action_idx])
            for seq_effect in seq_effects:
                if (
                    seq_effect[0] == parametrized_effect[0]
                    and seq_effect[2] == parametrized_effect[2]
                ):
                    effects_to_remove.append(seq_effect)
        for seq_effect in effects_to_remove:
            seq_effects.remove(seq_effect)

        # Add all effects of this action to the effect list
        for effect in action_descr["effects"]:
            parametrized_effect = parametrize_predicate(effect, parameters[action_idx])
            if parametrized_effect not in seq_effects:
                seq_effects.append(parametrized_effect)
    return seq_effects


def test_abstract_feasibility(knowledge_base, sequence, parameters, preconds):
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
        action_descr = knowledge_base.actions[action_id]

        # Check if any fact contradicts the pre-conditions of this action
        for fact in facts:
            for precond in action_descr["preconds"]:
                parametrized_precond = parametrize_predicate(
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
            parametrized_effect = parametrize_predicate(effect, parameters[action_idx])
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


def get_types_by_parent_type(types):
    types_by_parent = dict()
    for type_name in types:
        for parent_type_name in types[type_name]:
            if parent_type_name not in types_by_parent:
                types_by_parent[parent_type_name] = list()
            types_by_parent[parent_type_name].append(type_name)
    return types_by_parent
