import time
from copy import deepcopy

from highlevel_planning.learning.logic_tools import determine_sequence_effects


class PDDLExtender(object):
    def __init__(self, knowledge_base, predicates):
        self.predicates = predicates
        self.knowledge_base = knowledge_base

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
            self.knowledge_base, sequence, parameters
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

        self.knowledge_base.add_action(
            action_name, new_action_description, overwrite=False
        )

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

        # Full action parameters: list of tuples (name, type, value)
        full_action_params = list()
        for param_spec in action_params:
            full_action_params.append((param_spec[0], param_spec[1], param_spec[0]))

        # Add non-generalizable parameterization
        self._process_parameterizations(full_action_params, action_name)

        # Save action meta data
        self.knowledge_base.add_meta_action(
            action_name,
            sequence,
            parameters,
            param_translator,
            hidden_parameters,
            new_action_description,
        )

        return action_name

    def generalize_action(self, action_name, parameters):
        assert type(parameters) is dict
        action_description = self.knowledge_base.actions[action_name]
        types_of_params = {param[0]: param[1] for param in action_description["params"]}
        param_list = list()
        for parameter, object_name in parameters.items():
            if not object_name in self.knowledge_base.objects:
                self.knowledge_base.make_permanent(object_name)
            if not self.knowledge_base.is_type(
                object_to_check=object_name, type_query=types_of_params[parameter]
            ):
                self.knowledge_base.add_object(object_name, types_of_params[parameter])
            param_list.append([object_name, types_of_params[parameter]])
        self._process_parameterizations(param_list, action_name)

    def _retype_argument(self, arg, action_params, already_retyped, time_string):
        if arg not in already_retyped:
            if arg not in self.knowledge_base.objects:
                # Make arguments we use permanent
                self.knowledge_base.make_permanent(arg)
            if self.knowledge_base.is_type(arg, "position"):
                action_params.append([arg, "position"])
            else:
                original_types = self.knowledge_base.objects[arg]
                new_type = arg + "-" + time_string
                action_params.append([arg, new_type])
                self.knowledge_base.add_type(new_type, original_types[0])
                self.knowledge_base.add_object(arg, new_type)
            already_retyped.append(arg)

    def _process_parameterizations(self, param_list, action_name):
        """
        [summary]

        Args:
            param_list (list): List, where each element is a tuple (name, type, value)
            action_name (string): name of the action
        """
        object_params = list()
        for param in param_list:
            if param[1] != "position":
                object_params.append(param)
        object_params = tuple(object_params)
        for param in param_list:
            if param[1] == "position":
                if action_name not in self.knowledge_base.parameterizations:
                    self.knowledge_base.parameterizations[action_name] = dict()
                if (
                    object_params
                    not in self.knowledge_base.parameterizations[action_name]
                ):
                    self.knowledge_base.parameterizations[action_name][
                        object_params
                    ] = dict()
                if (
                    param[0]
                    not in self.knowledge_base.parameterizations[action_name][
                        object_params
                    ]
                ):
                    self.knowledge_base.parameterizations[action_name][object_params][
                        param[0]
                    ] = set()
                self.knowledge_base.parameterizations[action_name][object_params][
                    param[0]
                ].add(param[2])

    def create_new_predicates(self):
        pass
