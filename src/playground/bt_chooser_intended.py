#!/usr/bin/env python
#

# Copied from https://github.com/splintered-reality/py_trees/issues/50

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import py_trees.console as console
import time

##############################################################################
# Classes
##############################################################################


def description():
    content = "Investigate the chooser behaviour.\n\n"
    return content


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('-i', '--immediate-failure', action='store_const', dest="scenario", const="immediate-failure", help='immediate failure, falls back to the second priority')
    group.add_argument('-o', '--fail-after-one', action='store_const', dest="scenario", const="fail-after-one", help='fail after one, never falls back to the second priority')
    group.add_argument('-f', '--fail-after-one-then-immediate', action='store_const', dest="scenario", const="fail-after-one-then-immediate", help='fail after one and then replaces with an immediate failure')
    parser.set_defaults(scenario="immediate-failure")
    return parser


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


def create_tree(scenario):
    root = py_trees.composites.Chooser(name="Root")
    # running = py_trees.behaviours.Success(name="Running")
    success = py_trees.behaviours.Success(name="Success")
    if scenario == "immediate-failure":
        root.add_child(py_trees.behaviours.Failure(name="Failure"))
    elif scenario == "fail-after-one":
        root.add_child(FailAfterOne("FailAfterOne"))
    elif scenario == "fail-after-one-then-immediate":
        root.add_child(FailAfterOneThenImmediate(name="FailAfterOneThenImmediate"))
    root.add_child(success)
    return root

##############################################################################
# Main
##############################################################################


if __name__ == '__main__':
    """
    Entry point for the demo script.
    """
    args = command_line_argument_parser().parse_args()
    print(description())
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    ####################
    # Tree
    ####################
    tree = create_tree(args.scenario)
    tree.setup(timeout=15)

    ####################
    # Execute
    ####################
    for i in range(1, 5):
        try:
            print("\n--------- Tick {0} ---------\n".format(i))
            tree.tick_once()
            print("\n")
            print(py_trees.display.ascii_tree(tree, show_status=True))
            time.sleep(1.0)
        except KeyboardInterrupt:
            break
    print("\n")