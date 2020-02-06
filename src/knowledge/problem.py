class PlanningProblem:
    """
        For now this class just hardcodes the planning problem. Later, the object list should be populated
        through perception and the initial state (predicates) should be determined through observation.

        Only the goal will probably always be hard-coded.
    """

    def __init__(self):

        self.objects = [
            ("cube1", "object"),
            ("robot1", "chimera"),
            ("pos1", "position"),
        ]

        self.initial_predicates = [("empty-hand", "robot1")]

        self.goals = [("in-hand", False, ("cube1", "robot1"))]

        # self.goals = [("inside", False, ("cube1", "table"))]
