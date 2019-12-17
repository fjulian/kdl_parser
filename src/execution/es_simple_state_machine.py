from execution.es import ExecutionSystem


class SimpleStateMachine(ExecutionSystem):
    def __init__(self):
        self.ticking = False

    def step(self):
        pass

