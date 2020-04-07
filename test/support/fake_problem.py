class FakePlanningProblem:
    def __init__(self):

        self.objects = {
            "r1": "rover",
            "s1": "sample",
            "p1": "waypoint",
            "p2": "waypoint",
        }
        self.initial_predicates = [
            ("carry", True, ("r1", "s1",)),
            ("is-dropping-dock", True, ("p2",)),
        ]
        self.goals = [("new-predicate", True, ("p2", "r1"))]
