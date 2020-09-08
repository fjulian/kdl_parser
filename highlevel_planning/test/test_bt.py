import unittest
import py_trees

from highlevel_planning.execution.custom_chooser import CustomChooser


class TestCustomChooser(unittest.TestCase):
    def setUp(self):
        pass

    def test_default_chooser(self):
        chooser = py_trees.composites.Chooser(name="Default chooser")
        tree = create_tree(chooser, "immediate-failure")
        tick_four_times(tree)
        self.assertEqual(tree.status, py_trees.common.Status.SUCCESS)

        chooser = py_trees.composites.Chooser(name="Default chooser")
        tree = create_tree(chooser, "fail-after-one")
        tick_four_times(tree)
        self.assertEqual(tree.status, py_trees.common.Status.FAILURE)

        chooser = py_trees.composites.Chooser(name="Default chooser")
        tree = create_tree(chooser, "fail-after-one-then-immediate")
        tick_four_times(tree)
        self.assertEqual(tree.status, py_trees.common.Status.SUCCESS)

    def test_custom_chooser(self):
        chooser = CustomChooser(name="Custom chooser")
        tree = create_tree(chooser, "immediate-failure")
        tick_four_times(tree)
        self.assertEqual(tree.status, py_trees.common.Status.SUCCESS)

        chooser = CustomChooser(name="Custom chooser")
        tree = create_tree(chooser, "fail-after-one")
        tick_four_times(tree)
        self.assertEqual(tree.status, py_trees.common.Status.SUCCESS)

        chooser = CustomChooser(name="Custom chooser")
        tree = create_tree(chooser, "fail-after-one-then-immediate")
        tick_four_times(tree)
        self.assertEqual(tree.status, py_trees.common.Status.SUCCESS)


def tick_four_times(tree):
    for i in range(1, 5):
        print("\n--------- Tick {0} ---------\n".format(i))
        tree.tick_once()
        print("\n")
        py_trees.display.print_ascii_tree(tree, show_status=True)


class FailAfterOne(py_trees.behaviour.Behaviour):
    def __init__(self, name="Counter"):
        super(FailAfterOne, self).__init__(name)

    def initialise(self):
        self.counter = 0

    def update(self):
        self.counter += 1
        if self.counter == 1:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE


class FailAfterOneThenImmediate(py_trees.behaviour.Behaviour):
    def __init__(self, name="Counter"):
        super(FailAfterOneThenImmediate, self).__init__(name)
        self.counter = 0

    def update(self):
        self.counter += 1
        if self.counter == 1:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE


def create_tree(chooser, scenario):
    root = chooser
    success = py_trees.behaviours.Success(name="Success")
    if scenario == "immediate-failure":
        root.add_child(py_trees.behaviours.Failure(name="Failure"))
    elif scenario == "fail-after-one":
        root.add_child(FailAfterOne("FailAfterOne"))
    elif scenario == "fail-after-one-then-immediate":
        root.add_child(FailAfterOneThenImmediate(name="FailAfterOneThenImmediate"))
    root.add_child(success)
    return root


if __name__ == "__main__":
    unittest.main()
