import argparse
import pickle
import time

from multiprocessing import Lock

# Simulation
from sim.world import World
from sim.robot_arm import RobotArm
from sim.scene_planning_1 import ScenePlanning1

# Skills
from skills.navigate import ProcessNavigate
from skills.grasping import ProcessGrasping
from skills.placing import ProcessPlacing
from execution.bt import ExecutionSystem
from skills import pddl_descriptions
from knowledge.predicates import Predicates
from knowledge.problem import PlanningProblem

# Interface to BT
import py_trees

# Interface to planner and PDDL
from pddl_interface import pddl_file_if, planner_interface

# -------------------------------------------------------------


def main():
    # Command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r",
        "--reuse-objects",
        action="store_true",
        help="if given, the simulation does not reload objects. Objects must already be present.",
    )
    args = parser.parse_args()

    # Load existing simulation data if desired
    restore_existing_objects = args.reuse_objects
    objects = None
    robot_mdl = None
    if restore_existing_objects:
        with open("data/sim/objects.pkl", "rb") as pkl_file:
            objects, robot_mdl = pickle.load(pkl_file)

    # -----------------------------------

    # Set up planner interface and domain representation
    pddl_if = pddl_file_if.PDDLFileInterface(
        domain_dir="knowledge/chimera/domain",
        problem_dir="knowledge/chimera/problem",
        domain_name="chimera-domain",
    )
    temp = pddl_descriptions.get_action_description("grasp")
    pddl_if.add_action(action_name=temp[0], action_definition=temp[1], overwrite=False)
    temp = pddl_descriptions.get_action_description("nav")
    pddl_if.add_action(action_name=temp[0], action_definition=temp[1], overwrite=False)
    temp = pddl_descriptions.get_action_description("place")
    pddl_if.add_action(action_name=temp[0], action_definition=temp[1], overwrite=False)

    # -----------------------------------

    # Create world
    world = World(gui_=True, sleep_=True, load_objects=not restore_existing_objects)
    scene = ScenePlanning1(world, restored_objects=objects)

    # Spawn robot
    robot_lock = Lock()
    robot = RobotArm(world, robot_mdl)
    robot.reset()

    # -----------------------------------
    # Set up predicates

    preds = Predicates(scene, robot, robot_lock)
    for descr in preds.descriptions.items():
        pddl_if.add_predicate(
            predicate_name=descr[0], predicate_definition=descr[1], overwrite=False
        )

    planning_problem = PlanningProblem()
    pddl_if.add_objects(planning_problem.objects)
    pddl_if.add_inital_predicates(planning_problem.initial_predicates)
    pddl_if.add_goal(planning_problem.goals)

    pddl_if.save_domain()
    pddl_if.write_domain_pddl()
    pddl_if.write_problem_pddl()

    plan = planner_interface.pddl_planner(
        pddl_if._domain_file_pddl, pddl_if._problem_file_pddl
    )

    if plan is False:
        raise RuntimeError("Planning failed.")

    # -----------------------------------

    # Set up skills
    sk_grasp = ProcessGrasping(scene, robot, robot_lock)
    sk_nav = ProcessNavigate(scene, robot._model.uid)
    sk_place = ProcessPlacing(scene, robot, robot_lock)
    pipes = {"grasp": sk_grasp.get_pipe(), "nav": sk_nav.get_pipe(), "place": sk_place}

    # Set up behavior tree
    es = ExecutionSystem(
        robot, preds, plan=plan, goals=planning_problem.goals, pipes=pipes
    )
    py_trees.display.render_dot_tree(es.tree.root)
    es.setup()

    # -----------------------------------

    robot.to_start()
    world.step_seconds(0.5)

    try:
        # es.tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)

        index = 1
        while True:
            es.step()
            print("------------- Iteration {} ---------------".format(index))
            es.print_status()
            index += 1
            if es.ticking:
                time.sleep(0.5)

    except KeyboardInterrupt:
        # es.tree.interrupt()
        pass


if __name__ == "__main__":
    main()
