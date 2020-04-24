import argparse
import pickle
import time
import numpy as np

# Simulation
from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm
from highlevel_planning.sim.scene_planning_1 import ScenePlanning1

# Skills
from highlevel_planning.skills.navigate import SkillNavigate
from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.placing import SkillPlacing
from highlevel_planning.execution.es_sequential_execution import SequentialExecution
from highlevel_planning.skills import pddl_descriptions
from highlevel_planning.knowledge.predicates import Predicates

# Learning
from highlevel_planning.knowledge.knowledge_base import KnowledgeBase
from highlevel_planning.learning.explorer import Explorer
from highlevel_planning.learning.pddl_extender import PDDLExtender

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
    parser.add_argument(
        "-s",
        "--sleep",
        action="store_true",
        help="if given, the simulation will sleep for each update step, to mimic real time execution.",
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
    kb = KnowledgeBase("knowledge/chimera", domain_name="chimera-domain")

    # Add basic skill descriptions
    skill_descriptions = pddl_descriptions.get_action_descriptions()
    for skill_name, description in skill_descriptions.items():
        kb.add_action(
            action_name=skill_name, action_definition=description, overwrite=True,
        )

    # Add required types
    kb.add_type("robot")
    kb.add_type("navgoal")  # Anything we can navigate to
    kb.add_type("position", "navgoal")  # Pure positions
    kb.add_type("item", "navgoal")  # Anything we can grasp

    # Add origin
    kb.add_object("origin", "position", np.array([0.0, 0.0, 0.0]))
    kb.add_object("robot1", "robot")

    # -----------------------------------

    # Create world
    world = World(
        gui_=True, sleep_=args.sleep, load_objects=not restore_existing_objects
    )
    scene = ScenePlanning1(world, restored_objects=objects)

    # Spawn robot
    robot = RobotArm(world, robot_mdl)
    robot.reset()

    robot.to_start()
    world.step_seconds(0.5)

    # -----------------------------------

    # Set up predicates
    preds = Predicates(scene, robot)
    kb.set_predicate_funcs(preds)

    for descr in preds.descriptions.items():
        kb.add_predicate(
            predicate_name=descr[0], predicate_definition=descr[1], overwrite=True
        )

    # Planning problem
    kb.populate_objects(scene)
    kb.check_predicates()

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot)
    sk_place = SkillPlacing(scene, robot)
    sk_nav = SkillNavigate(scene, robot)
    skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

    # PDDL extender
    pddl_ex = PDDLExtender(kb, preds)

    # Set up exploration
    xplorer = Explorer(skill_set, robot._model.uid, scene.objects, pddl_ex, kb)

    # ---------------------------------------------------------------

    # Run planner
    plan = kb.solve()

    if plan is False:
        print("No plan found, start exploration")
        success = xplorer.exploration()
        if not success:
            print("Exploration was not successful")
            return

        # Run planner again
        plan = kb.solve()
        if plan is False:
            print("Planner failed despite exploration")
            return

    if len(plan) == 0:
        print("Nothing to do.")
        return
    print("Found plan:")
    print(plan)
    raw_input("Press enter to run...")

    # -----------------------------------

    # Set up execution system
    es = SequentialExecution(skill_set, plan, kb)

    # Run
    try:
        index = 1
        while True:
            print("------------- Iteration {} ---------------".format(index))
            es.print_status()
            success, plan_finished = es.step()
            index += 1
            if plan_finished:
                print("Plan finished. Exiting.")
                break
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
