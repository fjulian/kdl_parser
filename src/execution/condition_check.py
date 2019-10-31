import py_trees.common

class ConditionChecker(py_trees.behaviour.Behaviour):
    def __init__(self, checker_variable):
        super(ConditionChecker, self).__init__(name='checker_'+checker_variable)
        self._checker_variable = checker_variable
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        val = self.blackboard.get(self._checker_variable)
        if val:
            new_status = py_trees.common.Status.SUCCESS
            self.feedback_message = "Check succeeded"
        else:
            new_status = py_trees.common.Status.FAILURE
            self.feedback_message = "Check failed"
        self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        return new_status
