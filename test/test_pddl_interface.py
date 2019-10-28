import unittest

import sys
from os import path
sys.path.append( path.join(path.dirname( path.dirname( path.abspath(__file__) ) ), 'src') )

from pddl_interface.planner_interface import PDDLsolver
from pddl_interface.pddl_file_if import PDDLFileInterface

class TestInterface(unittest.TestCase):
    def setUp(self):
        self.slv = PDDLsolver()
        self.fif = PDDLFileInterface("knowledge/pddl-examples/rover/strips_typing/domain.pddl")

    def test_output(self):
        res = self.slv.plan()
        self.assertTrue(True)

    def test_cut_methods(self):
        self.assertEqual(self.slv.cut_string_before("Hello world. What's up?", "What's"), "What's up?")
        self.assertEqual(self.slv.cut_string_before("Hello world.", "What's"), "Hello world.")

        self.assertEqual(self.slv.cut_string_at("Hello world. What's up?", "world"), "Hello ")
        self.assertEqual(self.slv.cut_string_at("Hello world", "up"), "Hello world")

    def test_pddl_read_write(self):
        self.fif.read_domain()
        self.fif.write_domain()
        

if __name__ == '__main__':
    unittest.main()
