from highlevel_planning.execution.es import ExecutionSystem
from highlevel_planning.tools.util import SkillExecutionError
from highlevel_planning.learning.logic_tools import parametrize_predicate


class SequentialExecution(ExecutionSystem):
    def __init__(self, skill_set, plan, knowledge_base):
        self.ticking = False

        self.skill_set_ = skill_set
        self.plan = plan

        self.knowledge_base = knowledge_base

        self.current_idx_ = 0
        if len(plan) == 0:
            self.finished_plan = True
        else:
            self.finished_plan = False

        # Define ignore effects
        self.ignore_effects = {
            "nav-in-reach": [
                ("in-reach", False, ["current_pos", "rob"]),
                ("at", False, ["current_pos", "rob"]),
            ],
            "nav-at": [
                ("at", False, ["current_pos", "rob"]),
                ("in-reach", False, ["current_pos", "rob"]),
            ],
        }

    def step(self):
        success = True
        msgs = []
        if not self.finished_plan:
            plan_item = self.plan[self.current_idx_]
            plan_item_list = plan_item.split(" ")
            action_name = plan_item_list[1]
            action_name = action_name.split("_")[0]
            if len(plan_item_list) > 2:
                action_parameters = plan_item_list[2:]
            else:
                action_parameters = []
            success, msgs = self.execute_action(action_name, action_parameters)

            if success:
                self.current_idx_ += 1
                if self.current_idx_ == len(self.plan):
                    self.finished_plan = True
        return success, self.finished_plan, msgs

    def execute_action(self, action_name, action_parameters):
        msgs = []
        success = True
        if action_name in self.knowledge_base.meta_actions:
            expanded = self.knowledge_base.expand_step(action_name, action_parameters)
            for sub_step in expanded:
                ret_success, ret_msgs = self.execute_action(sub_step[0], sub_step[1])
                success &= ret_success
                msgs.extend(ret_msgs)
        else:
            try:
                if action_name == "grasp":
                    target_name = action_parameters[0]
                    target_link_id = -1
                    target_grasp_id = 0
                    res = self.skill_set_["grasp"].grasp_object(
                        target_name, target_link_id, target_grasp_id
                    )
                    if not res:
                        raise SkillExecutionError
                elif action_name == "nav-in-reach" or action_name == "nav-at":
                    target_name = action_parameters[1]
                    if self.knowledge_base.is_type(target_name, type_query="position"):
                        position = self.knowledge_base.lookup_table[target_name]
                        self.skill_set_["nav"].move_to_pos(position, nav_min_dist=0.3)
                    else:
                        self.skill_set_["nav"].move_to_object(target_name)
                elif action_name == "place":
                    target_pos_name = action_parameters[1]
                    target_pos = self.knowledge_base.lookup_table[target_pos_name]
                    self.skill_set_["place"].place_object(target_pos)
                else:
                    raise (
                        NotImplementedError,
                        "SequentialExecution script cannot deal with action "
                        + action_name
                        + " yet.",
                    )
            except SkillExecutionError:
                success = False
                msgs.append("Failure during execution of {}".format(action_name))

        # Check if the effects were reached successfully
        action_description = self.knowledge_base.actions[action_name]
        action_parameters_dict = {
            param[0]: action_parameters[idx]
            for idx, param in enumerate(action_description["params"])
        }
        for effect in action_description["effects"]:
            if action_name in self.ignore_effects:
                skip_effect = False
                for ignore_effect in self.ignore_effects[action_name]:
                    if ignore_effect == effect:
                        skip_effect = True
                if skip_effect:
                    continue
            parameterized_effect = parametrize_predicate(effect, action_parameters_dict)
            res = self.knowledge_base.predicate_funcs.call[effect[0]](
                *parameterized_effect[2]
            )
            if not res == effect[1]:
                success = False
                msgs.append(
                    "Failed to reach effect {} during action {}".format(
                        effect[0], action_name
                    )
                )

        return success, msgs

    def print_status(self):
        if not self.finished_plan:
            print("Plan item up next: " + self.plan[self.current_idx_])
        else:
            print("Finished plan")
