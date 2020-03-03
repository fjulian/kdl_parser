from highlevel_planning.execution.es import ExecutionSystem


class SequentialExecution(ExecutionSystem):
    def __init__(self, skill_set, plan):
        self.ticking = False

        self.skill_set_ = skill_set
        self.plan_ = plan

        self.current_idx_ = 0
        self.finished_plan_ = False

    def step(self):
        if not self.finished_plan_:
            plan_item = self.plan_[self.current_idx_]
            plan_item_list = plan_item.split(" ")
            action_name = plan_item_list[1]

            if action_name == "grasp":
                target_name = plan_item_list[2]
                target_link_id = None
                target_grasp_id = 0
                self.skill_set_["grasp"].grasp_object(
                    target_name, target_link_id, target_grasp_id
                )
            elif action_name == "nav":
                target_name = plan_item_list[2]
                self.skill_set_["nav"].move_to_object(target_name)
            elif action_name == "place":
                target_pos = plan_item_list[2]
                self.skill_set_["place"].place_object(target_pos)
            else:
                raise (
                    NotImplementedError,
                    "SequentialExecution script cannot deal with action "
                    + action_name
                    + " yet.",
                )

            self.current_idx_ += 1
            if self.current_idx_ == len(self.plan_):
                self.finished_plan_ = True
        return self.finished_plan_

    def print_status(self):
        if not self.finished_plan_:
            print("Plan item up next: " + self.plan_[self.current_idx_])
        else:
            print("Finished plan")
