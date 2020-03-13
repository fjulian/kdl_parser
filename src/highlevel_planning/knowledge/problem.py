class PlanningProblem:
    """
        For now this class just hardcodes the planning problem. Later, the object list should be populated
        through perception and the initial state (predicates) should be determined through observation.

        Only the goal will probably always be hard-coded.
    """

    def __init__(self):

        self.objects = list()
        self.initial_predicates = list()

        # self.goals = [("in-hand", True, ("cube1", "robot1"))]
        self.goals = [("on", True, ("cupboard", "cube1"))]
        # self.goals = [("inside", True, ("container1", "cube1"))]

    def populate_objects(self, scene=None, knowledge_lookups=None):
        if scene is not None:
            for obj in scene.objects:
                self.objects.append((obj, "item"))
        if knowledge_lookups is not None:
            for cat in knowledge_lookups:
                for item in knowledge_lookups[cat].data:
                    self.objects.append((item, cat))

        # Remove duplicates
        self.objects = list(dict.fromkeys(self.objects))

    def check_predicates(self, predicates):
        """
        If predicates need to be initialized when the system is launched, this can be done here.
        
        Args:
            predicates ([type]): [description]
            robot ([type]): [description]
        """

        if predicates.empty_hand("robot1"):
            self.initial_predicates.append(("empty-hand", "robot1"))
        self.initial_predicates.append(("in-reach", "origin", "robot1"))

        # Check any predicates in relation with the goal
        for goal in self.goals:
            if predicates.call[goal[0]](*goal[2]):
                pred_tuple = (goal[0],) + goal[2]
                self.initial_predicates.append(pred_tuple)

    def test_goals(self, predicates):
        for goal in self.goals:
            if not predicates.call[goal[0]](*goal[2]):
                return False
        return True
