import argparse
import pickle
import time
from multiprocessing import Lock
import numpy as np

# Simulation
from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm
from highlevel_planning.sim.scene_planning_1 import ScenePlanning1

# Skills
from highlevel_planning.skills.navigate import ProcessNavigate, SkillNavigate
from highlevel_planning.skills.grasping import ProcessGrasping, SkillGrasping
from highlevel_planning.skills.placing import ProcessPlacing, SkillPlacing
from highlevel_planning.execution.es_behavior_tree import AutoBehaviourTree
from highlevel_planning.execution.es_sequential_execution import SequentialExecution
from highlevel_planning.skills import pddl_descriptions
from highlevel_planning.knowledge.predicates import Predicates
from highlevel_planning.knowledge.problem import PlanningProblem
from highlevel_planning.knowledge.lookup_tables import LookupTable

# Learning
from highlevel_planning.learning.explorer import Explorer

# Interface to planner and PDDL
from highlevel_planning.pddl_interface import pddl_file_if, planner_interface

# ----------------------------------------------------------------------


def main():
    # Seed RNGs
    np.random.seed(0)

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
        domain_dir="knowledge/chimera/main/domain",
        problem_dir="knowledge/chimera/main/problem",
        domain_name="chimera-domain",
    )

    # Add skill descriptions
    skill_descriptions = pddl_descriptions.get_action_descriptions()
    for skill_name in skill_descriptions:
        pddl_if.add_action(
            action_name=skill_name,
            action_definition=skill_descriptions[skill_name],
            overwrite=True,
        )

    # Add required types
    pddl_if.add_type("robot")
    pddl_if.add_type("navgoal")  # Anything we can navigate to
    pddl_if.add_type("position", "navgoal")  # Pure positions
    pddl_if.add_type("item", "navgoal")  # Anything we can grasp

    # Set up knowledge data structure
    knowledge_lookups = dict()
    knowledge_lookups["position"] = LookupTable("position")
    knowledge_lookups["position"].add("origin", np.array([0.0, 0.0, 0.0]))

    # -----------------------------------

    # Create world
    world = World(gui_=True, sleep_=False, load_objects=not restore_existing_objects)
    scene = ScenePlanning1(world, restored_objects=objects)

    # Spawn robot
    robot_lock = Lock()
    robot = RobotArm(world, robot_mdl)
    robot.reset()

    robot.to_start()
    world.step_seconds(0.5)

    knowledge_lookups["robot"] = LookupTable("robot")
    knowledge_lookups["robot"].add("robot1", robot)

    # -----------------------------------
    # Set up predicates

    preds = Predicates(scene, robot, knowledge_lookups, robot_lock)

    for descr in preds.descriptions.items():
        pddl_if.add_predicate(
            predicate_name=descr[0], predicate_definition=descr[1], overwrite=True
        )

    planning_problem = PlanningProblem()
    planning_problem.populate_objects(scene, knowledge_lookups)
    planning_problem.check_predicates(preds)

    pddl_if.add_planning_problem(planning_problem)

    pddl_if.save_domain()
    pddl_if.write_domain_pddl()
    pddl_if.write_problem_pddl()

    plan = planner_interface.pddl_planner(
        pddl_if._domain_file_pddl, pddl_if._problem_file_pddl
    )

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot)
    sk_place = SkillPlacing(scene, robot)
    sk_nav = SkillNavigate(scene, robot)
    skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

    # Set up exploration
    xplorer = Explorer(
        pddl_if,
        planning_problem,
        skill_set,
        knowledge_lookups,
        robot._model.uid,
        scene.objects,
    )

    if plan is False:
        xplorer.exploration(preds)
        return
    else:
        if len(plan) == 0:
            print("Nothing to do.")
            return
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
    else:
        es = SequentialExecution(skill_set, plan, knowledge_lookups)
    es.setup()

    # -----------------------------------

    # Run
    try:
        index = 1
        while True:
            print("------------- Iteration {} ---------------".format(index))
            es.print_status()
            success, plan_finished = es.step()
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
