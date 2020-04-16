import unittest

import sys
from os import path, remove


from highlevel_planning.pddl_interface.planner_interface import (
    pddl_planner,
    cut_string_at,
    cut_string_before,
)
from highlevel_planning.pddl_interface.pddl_file_if import PDDLFileInterface


class TestInterface(unittest.TestCase):
    def setUp(self):
        self.fif_domain_dir = "test/data"
        self.fif_init_domain_name = "test_domain.pddl"
        self.fif = PDDLFileInterface(
            self.fif_domain_dir, initial_domain_pddl=self.fif_init_domain_name
        )

    def test_output(self):
        res = pddl_planner(
            "test/data/examples/strips_typing_valid/domain.pddl",
            "test/data/examples/strips_typing_valid/problem.pddl",
        )
        self.assertEqual(res[0], "0: move rover1 waypoint6 waypoint7")
        self.assertEqual(len(res), 66)
        self.assertEqual(res[-1], "65: move rover1 waypoint9 waypoint1")

        res = pddl_planner(
            "test/data/examples/strips_typing_invalid/domain.pddl",
            "test/data/examples/strips_typing_invalid/problem.pddl",
        )
        self.assertFalse(res)

    def test_cut_methods(self):
        self.assertEqual(
            cut_string_before("Hello world. What's up?", "What's"), "What's up?"
        )
        self.assertEqual(cut_string_before("Hello world.", "What's"), "Hello world.")

        self.assertEqual(cut_string_at("Hello world. What's up?", "world"), "Hello ")
        self.assertEqual(cut_string_at("Hello world", "up"), "Hello world")

    def test_pddl_read_write(self):
        self.assertEqual(len(self.fif._predicates), 11)
        self.assertEqual(len(self.fif._actions), 4)

        self.fif.write_domain_pddl()

        # with open(path.join(self.fif_domain_dir, self.fif_init_domain_name), "r") as f:
        #     orig_pddl = f.read()
        # with open(self.fif._domain_file_pddl, "r") as f:
        #     new_pddl = f.read()
        # Unfortunately cannot directly compare these, since dicts don't keep order
        # self.assertEqual(orig_pddl, new_pddl)

        self.fif.save_domain()
        fif2 = PDDLFileInterface(self.fif_domain_dir)
        self.assertEqual(self.fif._domain_name, fif2._domain_name)
        self.assertEqual(self.fif._predicates, fif2._predicates)
        self.assertEqual(self.fif._actions, fif2._actions)
        self.assertEqual(self.fif._types, fif2._types)

        remove(self.fif._domain_file_pddl)
        remove(self.fif._domain_file)

    def test_pddl_add_items(self):
        self.fif.add_action(
            "dummy_action",
            {"params": [], "preconds": [], "effects": [("pred1", False, [])]},
        )
        self.assertTrue("dummy_action" in self.fif._actions)
        self.assertTrue(isinstance(self.fif._actions["dummy_action"], dict))
        self.assertFalse(self.fif._actions["dummy_action"]["effects"][0][1])

        self.assertTrue(len(self.fif._actions["dummy_action"]) > 0)
        self.fif.add_action("dummy_action", {}, overwrite=False)
        self.assertTrue(len(self.fif._actions["dummy_action"]) > 0)
        self.fif.add_action("dummy_action", {}, overwrite=True)
        self.assertFalse(len(self.fif._actions["dummy_action"]) > 0)


if __name__ == "__main__":
    unittest.main()
