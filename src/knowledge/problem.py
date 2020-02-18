class PlanningProblem:
    """
        For now this class just hardcodes the planning problem. Later, the object list should be populated
        through perception and the initial state (predicates) should be determined through observation.

        Only the goal will probably always be hard-coded.
    """

    def __init__(self):

        self.objects = [
            ("robot1", "chimera"),
            ("pos1", "position"),
        ]

        self.initial_predicates = []

        self.goals = [("in-hand", False, ("cube1", "robot1"))]
        # self.goals = [("inside", False, ("container1", "cube1"))]

    def populate_objects(self, scene):
        for obj in scene.objects:
            self.objects.append((obj, "object"))

    def check_predicates(self, predicates, robot):
        # If predicates need to be initialized when the system is launched, this can be done here.

        if predicates.empty_hand(robot):
            self.initial_predicates.append(("empty-hand", "robot1"))
