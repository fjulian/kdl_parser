import time
from copy import deepcopy

from highlevel_planning.learning.logic_tools import determine_sequence_effects


class PDDLExtender(object):
    def __init__(self, pddl_if, predicates, meta_action_handler, knowledge_lookups):
        self.pddl_if = pddl_if
        self.predicates = predicates
        self.meta_action_handler = meta_action_handler
        self.knowledge_lookups = knowledge_lookups

    def create_new_action(self, goals, sequence, parameters, sequence_preconds):
        time_string = time.strftime("%y%m%d%H%M%S")
        action_name = sequence[0] + "-" + goals[0][0] + "-" + time_string

        # Compute parameters
        action_params = list()
        already_retyped = list()

        # Compute effects
        action_effects = list()
        for goal in goals:
            for arg in goal[2]:
                self._retype_argument(arg, action_params, already_retyped, time_string)
            action_effects.append(goal)

        sequence_effects = determine_sequence_effects(
            self.pddl_if, sequence, parameters
        )
        for effect in sequence_effects:
            for arg in effect[2]:
                self._retype_argument(arg, action_params, already_retyped, time_string)
            action_effects.append(effect)

        # Compute preconditions
        action_preconditions = list()
        for precond in sequence_preconds:
            for arg in precond[2]:
                self._retype_argument(arg, action_params, already_retyped, time_string)
            action_preconditions.append(precond)

        # Submit new action description
        new_action_description = {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        }

        self.pddl_if.add_action(action_name, new_action_description)

        # Determine hidden parameters
        hidden_parameters = [{}] * len(parameters)
        for idx, params in enumerate(parameters):
            for param_name, param_value in params.items():
                if param_value not in already_retyped:
                    hidden_parameters[idx][param_name] = param_value

        # Determine translation between old argument names and new ones
        param_translator = [dict.fromkeys(param_dict) for param_dict in parameters]
        for idx, params in enumerate(parameters):
            for param_name, param_value in params.items():
                if param_value in already_retyped:
                    param_translator[idx][param_name] = param_value
                else:
                    param_translator[idx][param_name] = param_name

        # Save action meta data
        self.meta_action_handler.add_meta_action(
            action_name,
            sequence,
            parameters,
            param_translator,
            hidden_parameters,
            new_action_description,
        )

        return action_name

    def _retype_argument(self, arg, action_params, already_retyped, time_string):
        if arg not in already_retyped:
            original_types = None
            if arg in self.pddl_if._objects:
                original_types = self.pddl_if._objects[arg]
            else:
                for knowledge_type in self.knowledge_lookups:
                    if arg in self.knowledge_lookups[knowledge_type].data:
                        original_types = [knowledge_type]
                        break
            if original_types is None:
                raise RuntimeError("Unknown argument cannot be processed")
            new_type = arg + "-" + time_string
            action_params.append([arg, new_type])
            if original_types is not None:
                self.pddl_if.add_type(new_type, original_types[0])
            else:
                self.pddl_if.add_type(new_type)
            self.pddl_if.add_objects({arg: new_type}, update_existing=True)
            already_retyped.append(arg)

    def _get_unique_param_names(self, sequence):
        # Not used right now. Remove if still not used in the future.
        substitute_param_names = list()
        used_param_names = list()
        for action_name in sequence:
            substitutes = dict()
            params = self.pddl_if._actions[action_name]["params"]
            for param in params:
                if param[0] not in used_param_names:
                    used_param_names.append(param[0])
                    substitutes[param[0]] = param[0]
                else:
                    counter = 1
                    while param[0] + str(counter) in used_param_names:
                        counter += 1
                    assert (
                        counter < 10
                    ), "This implementation assumes that there are max 9 clashing parameters."
                    new_param_name = param[0] + str(counter)
                    used_param_names.append(new_param_name)
                    substitutes[param[0]] = new_param_name
            substitute_param_names.append(substitutes)
        return substitute_param_names

    def create_new_predicates(self):
        pass
