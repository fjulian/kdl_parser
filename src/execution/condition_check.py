import py_trees.common

class ConditionChecker_Blackboard(py_trees.behaviour.Behaviour):
    def __init__(self, checker_variable):
        super(ConditionChecker_Blackboard, self).__init__(name='checker_'+checker_variable)
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

class ConditionChecker_Predicate(py_trees.behaviour.Behaviour):
    def __init__(self, predicate_fcn, predicate_args=None):
        super(ConditionChecker_Predicate, self).__init__(name='checker_pred_'+predicate_fcn.__name__)
        self._predicate_fcn = predicate_fcn
        self._predicate_args = predicate_args
        if predicate_args is None:
            self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if self._predicate_args is None:
            args = self.blackboard.get(self._predicate_fcn.__name__+"_args")
        else:
            args = self._predicate_args
        res = self._predicate_fcn(*args)
        if res:
            new_status = py_trees.common.Status.SUCCESS
            self.feedback_message = "Check succeeded"
        else:
            new_status = py_trees.common.Status.FAILURE
            self.feedback_message = "Check failed"
        self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        return new_status
