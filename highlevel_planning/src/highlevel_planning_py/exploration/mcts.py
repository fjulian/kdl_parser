import pybullet as pb
from time import time
import numpy as np
from collections import defaultdict

from highlevel_planning_py.execution.es_sequential_execution import (
    execute_plan_sequentially,
)

MAX_DEPTH = 30


class HLPTreeSearch:
    def __init__(self, root):
        self.root = root

    def tree_search(self):
        time_budget = 700
        start_time = time()
        current_node = self.root
        while time() - start_time < time_budget:
            while not current_node.is_terminal():
                # current_node.receive_visit()
                if current_node.check_expanding():
                    # Expand
                    current_node = current_node.expand()
                    break
                else:
                    # Select a child to continue from
                    current_node = current_node.select_child()
            result = current_node.rollout()
            current_node.backpropagate(result)


class HLPTreeNode:
    def __init__(self, state, explorer, parent=None):
        self.state = state
        self.exp = explorer
        self.parent = parent

        self.action_list = [
            act
            for act in self.exp.knowledge_base.actions
            if act not in self.exp.knowledge_base.meta_actions
        ]
        self.children = list()
        self.child_actions = list()

        self.num_visited = 0
        self.never_expand = False
        self.results = defaultdict(int)

    # @property
    # def q(self):
    #     success = self._results[1]
    #     return success
    #
    # def expand(self):
    #     action = self.untried_actions.pop()
    #     next_state = self.state.move(action)
    #     child_node = HLPTreeNode(next_state, parent=self)
    #     self.children.append(child_node)
    #     return child_node

    # def receive_visit(self):
    #     self.num_visited += 1

    def is_terminal(self):
        self.state.is_game_over()

    def check_expanding(self):
        alpha = 0.4
        return np.floor(self.num_visited ** alpha) > np.floor(
            (self.num_visited - 1) ** alpha
        )

    def expand(self):
        max_tries = 50
        counter = 0
        while counter < max_tries:
            counter += 1
            sequence_tuple = self._sample_step()
            if sequence_tuple in self.child_actions:
                continue
            new_state = self.state.move(sequence_tuple)
            new_child = HLPTreeNode(new_state, self.exp, self)
            self.children.append(new_child)
            self.child_actions.append(sequence_tuple)
            return new_child
        return None

    def _sample_step(self):
        sequence = self.exp._sample_sequence(length=1)
        parameters = self.exp._sample_parameters(sequence)
        sequence_tuple = (tuple(sequence), tuple(parameters))
        return sequence_tuple

    def rollout(self):
        current_rollout_state = self.state
        while not current_rollout_state.is_game_over():
            max_tries = 50
            counter = 0
            new_state = None
            while counter < max_tries:
                counter += 1
                sequence_tuple = self._sample_step()
                new_state = current_rollout_state.move(sequence_tuple)
                if new_state.success:
                    break
            current_rollout_state = new_state
        return current_rollout_state.game_result

    def select_child(self, exploration_constant=np.sqrt(2)):
        scores = list()
        for child in self.children:
            avg_result = float(child.results[1]) / float(child.num_visited)
            score = avg_result + exploration_constant * np.sqrt(
                np.log(self.num_visited) / float(child.num_visited)
            )
            scores.append(score)
        max_idx = np.argmax(scores)
        return self.children[max_idx]

    def backpropagate(self, result):
        self.num_visited += 1
        self.results[result] += 1
        if self.parent:
            self.parent.backpropagate(result)


class HLPState:
    def __init__(self, success, depth, pb_client_id, explorer):
        self._depth = depth
        self._bullet_state = pb.saveState(physicsClientId=pb_client_id)
        self._bullet_client_id = pb_client_id

        self.exp = explorer
        self.success = success

    @property
    def game_result(self):
        if not self.success:
            return 0
        elif self._depth >= MAX_DEPTH:
            return 0
        elif self.goal_reached():
            return 1
        else:
            return None

    def is_game_over(self):
        return self.game_result is not None

    def move(self, action):
        self._restore_state()
        success = execute_plan_sequentially(
            action[0], action[1], self.exp.skill_set, self.exp.knowledge_base
        )
        new_state = HLPState(success, self._depth + 1, self._bullet_client_id, self.exp)
        return new_state

    # def get_legal_actions(self):
    #     self._restore_state()

    def _restore_state(self):
        pb.restoreState(
            stateId=self._bullet_state, physicsClientId=self._bullet_client_id
        )

    def goal_reached(self):
        self._restore_state()
        return self.exp.knowledge_base.test_goals()
