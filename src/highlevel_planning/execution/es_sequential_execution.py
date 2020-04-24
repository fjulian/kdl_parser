from highlevel_planning.execution.es import ExecutionSystem
from highlevel_planning.tools.util import SkillExecutionError


class SequentialExecution(ExecutionSystem):
    def __init__(self, skill_set, plan, knowledge_base):
        self.ticking = False

        self.skill_set_ = skill_set
        self.plan_ = knowledge_base.expand_plan(plan)

        self.knowledge_base = knowledge_base

        self.current_idx_ = 0
        if len(plan) == 0:
            self.finished_plan_ = True
        else:
            self.finished_plan_ = False

    def step(self):
        success = True
        if not self.finished_plan_:
            plan_item = self.plan_[self.current_idx_]
            plan_item_list = plan_item.split(" ")
            action_name = plan_item_list[1]

            try:
                if action_name == "grasp":
                    target_name = plan_item_list[2]
                    target_link_id = None
                    target_grasp_id = 0
                    res = self.skill_set_["grasp"].grasp_object(
                        target_name, target_link_id, target_grasp_id
                    )
                    if not res:
                        raise SkillExecutionError
                elif action_name == "nav":
                    target_name = plan_item_list[3]
                    if self.knowledge_base.is_type(target_name, type_query="position"):
                        position = self.knowledge_base.lookup_table[target_name]
                        self.skill_set_["nav"].move_to_pos(position, nav_min_dist=0.3)
                    else:
                        self.skill_set_["nav"].move_to_object(target_name)
                elif action_name == "place":
                    target_pos_name = plan_item_list[3]
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

            if success:
                self.current_idx_ += 1
                if self.current_idx_ == len(self.plan_):
                    self.finished_plan_ = True
        return success, self.finished_plan_

    def print_status(self):
        if not self.finished_plan_:
            print("Plan item up next: " + self.plan_[self.current_idx_])
        else:
            print("Finished plan")
