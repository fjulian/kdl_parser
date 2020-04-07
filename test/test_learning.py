import unittest
from highlevel_planning.pddl_interface.pddl_file_if import PDDLFileInterface
from highlevel_planning.learning import logic_tools
from highlevel_planning.learning.pddl_extender import PDDLExtender
from highlevel_planning.learning.meta_action_handler import MetaActionHandler
from support.fake_problem import FakePlanningProblem
from support.fake_predicates import FakePredicates

# from highlevel_planning.learning.explorer import Explorer


class TestLearning(unittest.TestCase):
    def setUp(self):
        self.fif_domain_dir = "test/data"
        self.fif_init_domain_name = "test_domain.pddl"
        self.fif = PDDLFileInterface(
            self.fif_domain_dir, initial_domain_pddl=self.fif_init_domain_name
        )

        self.planning_problem = FakePlanningProblem()
        self.fif.add_planning_problem(self.planning_problem)
        # self.explorer = Explorer(
        #     self.fif, self.planning_problem, None, None, None, None
        # )

        self.sequence = ["move", "drop-sample"]
        self.parameters = [
            {"from-waypoint": "p1", "rover": "r1", "to-waypoint": "p2"},
            {"rover": "r1", "sample": "s1", "waypoint": "p1"},
        ]
        self.sequence_preconds = logic_tools.determine_sequence_preconds(
            self.fif, self.sequence, self.parameters
        )

        self.predicates = FakePredicates()

        self.meta_action_handler = MetaActionHandler(self.fif)

        self.pddl_extender = PDDLExtender(
            self.fif, self.predicates, self.meta_action_handler
        )

    def test_new_action(self):
        new_action_name = self.pddl_extender.create_new_action(
            self.planning_problem.goals,
            self.sequence,
            self.parameters,
            self.sequence_preconds,
        )

        plan = ["0: take-sample r1 s1 p1", "1: " + new_action_name + " p2 r1 p1 s1"]
        expanded_plan = self.meta_action_handler.expand_plan(plan)

        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()
