class MetaActionHandler(object):
    def __init__(self, knowledge_base):
        self.knowledge_base = knowledge_base
        self.meta_actions = dict()

    def expand_plan(self, plan):
        expanded_plan = list()

        current_idx = 0

        for plan_item in plan:
            plan_item_list = plan_item.split(" ")
            action_name = plan_item_list[1]
            if len(plan_item_list) > 2:
                action_parameters = plan_item_list[2:]
            else:
                action_parameters = []
            if action_name in self.meta_actions:
                meta_action = self.meta_actions[action_name]
                parameter_order = [
                    param[0] for param in meta_action["description"]["params"]
                ]
                for idx, sub_action_name in enumerate(meta_action["seq"]):
                    new_plan_item = str(current_idx) + ": " + sub_action_name + " "
                    sub_action_parameters = self.knowledge_base.actions[
                        sub_action_name
                    ]["params"]
                    for param_spec in sub_action_parameters:
                        old_param_name = param_spec[0]
                        if old_param_name in meta_action["hidden_params"][idx]:
                            new_plan_item += (
                                meta_action["hidden_params"][idx][old_param_name] + " "
                            )
                        elif old_param_name in meta_action["param_translator"][idx]:
                            new_param_name = meta_action["param_translator"][idx][
                                old_param_name
                            ]
                            parameter_idx = parameter_order.index(new_param_name)
                            parameter_value = action_parameters[parameter_idx]
                            new_plan_item += parameter_value + " "
                        else:
                            raise RuntimeError(
                                "Parameter for sub action of meta action undefined"
                            )
                    new_plan_item = new_plan_item.strip()
                    expanded_plan.append(new_plan_item)
                    current_idx += 1
            else:
                new_plan_item = (
                    str(current_idx) + ": " + plan_item.split(":")[1].strip()
                )
                expanded_plan.append(new_plan_item)
                current_idx += 1
        return expanded_plan

    def add_meta_action(
        self,
        name,
        sequence,
        parameters,
        param_translator,
        hidden_parameters,
        description,
    ):
        assert type(name) is str
        assert type(sequence) is list
        self.meta_actions[name] = {
            "seq": sequence,
            "params": parameters,
            "param_translator": param_translator,
            "hidden_params": hidden_parameters,
            "description": description,
        }
