import py_trees
import time

class ConditionChecker_Blackboard(py_trees.behaviour.Behaviour):
    def __init__(self, checker_variable, duration=0):
        super(ConditionChecker_Blackboard, self).__init__(name='checker_'+checker_variable)
        self._checker_variable = checker_variable
        self._duration = duration
        self.blackboard = py_trees.blackboard.Blackboard()

    def initialise(self):
        self.time_left = self._duration

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        self.feedback_message = "Check running"
        if self.time_left > 0:
            self.time_left -= 1
        else:
            val = self.blackboard.get(self._checker_variable)
            if val:
                new_status = py_trees.common.Status.SUCCESS
                self.feedback_message = "Check succeeded"
            else:
                new_status = py_trees.common.Status.FAILURE
                self.feedback_message = "Check failed"
        self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        return new_status


def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("fail_var", False)
    blackboard.set("succ_var", True)
    blackboard.set("succ_var1", True)

    action_fail = ConditionChecker_Blackboard("fail_var", duration=2)
    action_fail2 = ConditionChecker_Blackboard("succ_var1")
    seq_fail = py_trees.composites.Sequence(children=[action_fail, action_fail2])
    action_success = ConditionChecker_Blackboard("succ_var")
    root = py_trees.composites.Chooser(children=[seq_fail, action_success])
    tree = py_trees.trees.BehaviourTree(root)

    print("="*20)
    print("Behavior tree:")
    print("-"*20)
    print(py_trees.display.ascii_tree(tree.root))
    print("="*20)

    for i in range(6):
        tree.tick()
        time.sleep(0.5)


if __name__ == '__main__':
    main()
