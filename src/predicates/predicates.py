


class Predicates:
    def __init__(self):
        self.call = {
            "empty-hand": self.empty_hand,
            "in-hand": self.in_hand,
            "in-reach": self.in_reach
        }

        self.descriptions = [
            ("empty-hand", [["rob", "chimera"]]),
            ("in-hand", [["obj", "object"], ["rob", "chimera"]]),
            ("in-reach", [["obj", "object"], ["rob", "chimera"]])
        ]

    def empty_hand(self, robot):
        pass

    def in_hand(self, object, robot):
        pass

    def in_reach(self, object, robot):
        pass
