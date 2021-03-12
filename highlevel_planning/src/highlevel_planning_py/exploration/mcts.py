import pybullet as pb

from mctspy.tree.nodes import TwoPlayersGameMonteCarloTreeSearchNode
from mctspy.games.common import TwoPlayersAbstractGameState


MAX_DEPTH = 30


class HLPState(TwoPlayersAbstractGameState):
    def __init__(self, depth, pb_client_id):
        self._depth = depth
        self._bullet_state = pb.saveState(physicsClientId=pb_client_id)
        self._bullet_client_id = pb_client_id

    @property
    def game_result(self):
        # TODO
        return 0

    def is_game_over(self):
        return self._depth >= MAX_DEPTH or self.game_result == 1

    def move(self, action):
        # TODO
        pass

    def get_legal_actions(self):
        self._restore_state()

    def _restore_state(self):
        pb.restoreState(
            stateId=self._bullet_state, physicsClientId=self._bullet_client_id
        )


class HLPTreeNode(TwoPlayersGameMonteCarloTreeSearchNode):
    def __init__(self, state, parent=None):
        super().__init__(state, parent)

    @property
    def q(self):
        success = self._results[1]
        return success

    def expand(self):
        action = self.untried_actions.pop()
        next_state = self.state.move(action)
        child_node = HLPTreeNode(next_state, parent=self)
        self.children.append(child_node)
        return child_node
