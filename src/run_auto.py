import argparse
import pickle
import time

from multiprocessing import Lock

# Simulation
from sim.world import World
from sim.robot_arm import RobotArm
from sim.scene_planning_1 import ScenePlanning1

# Skills
from skills.navigate import ProcessNavigate, SkillNavigate
from skills.grasping import ProcessGrasping, SkillGrasping
from skills.placing import ProcessPlacing, SkillPlacing
from execution.es_behavior_tree import AutoBehaviourTree
from execution.es_sequential_execution import SequentialExecution
from skills import pddl_descriptions
from knowledge.predicates import Predicates
from knowledge.problem import PlanningProblem

# Interface to BT
import py_trees

# Interface to planner and PDDL
from pddl_interface import pddl_file_if, planner_interface

# ----------------------------------------------------------------------


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

    robot.to_start()
    world.step_seconds(0.5)

    # -----------------------------------
    # Set up predicates

    preds = Predicates(scene, robot, robot_lock)

    for descr in preds.descriptions.items():
        pddl_if.add_predicate(
            predicate_name=descr[0], predicate_definition=descr[1], overwrite=False
        )

    planning_problem = PlanningProblem()
    planning_problem.populate_objects(scene)
    planning_problem.check_predicates(preds, robot)

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
    else:
        print("Found plan:")
        print(plan)
        raw_input("Press enter to run...")

    # -----------------------------------

    # Set up execution system
    use_bt = False
    if use_bt:
        # Set up skills
        sk_grasp = ProcessGrasping(scene, robot, robot_lock)
        sk_nav = ProcessNavigate(scene, robot._model.uid)
        sk_place = ProcessPlacing(scene, robot, robot_lock)
        pipes = {
            "grasp": sk_grasp.get_pipe(),
            "nav": sk_nav.get_pipe(),
            "place": sk_place,
        }

        # Set up behavior tree
        es = AutoBehaviourTree(
            robot, preds, plan=plan, goals=planning_problem.goals, pipes=pipes
        )
        # py_trees.display.render_dot_tree(es.tree.root)
    else:
        # Set up skills
        sk_grasp = SkillGrasping(scene, robot)
        sk_place = SkillPlacing(scene, robot)
        sk_nav = SkillNavigate(scene, robot._model.uid)
        skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

        es = SequentialExecution(skill_set, plan)
    es.setup()

    # -----------------------------------

    try:
        index = 1
        while True:
            print("------------- Iteration {} ---------------".format(index))
            es.print_status()
            plan_finished = es.step()
            index += 1
            if es.ticking:
                time.sleep(0.5)
            if plan_finished:
                print("Plan finished. Exiting.")
                break
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
