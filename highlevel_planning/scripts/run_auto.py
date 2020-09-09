from __future__ import print_function

import argparse
import pickle
import numpy as np
import os
import pybullet as p

try:
    input = raw_input
except NameError:
    pass

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

# Other
from highlevel_planning.tools.config import ConfigYaml

# ----------------------------------------------------------------------

BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


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
    parser.add_argument(
        "-m",
        "--method",
        action="store",
        help="determines in which mode to connect to pybullet. Can be 'gui', 'direct' or 'shared'.",
        default="gui"
    )
    args = parser.parse_args()

    if args.method == "direct" and args.reuse_objects:
        raise RuntimeError("Cannot reload objects when in direct mode.")

    # Load existing simulation data if desired
    restore_existing_objects = args.reuse_objects
    objects = None
    robot_mdl = None
    if restore_existing_objects:
        with open(
            os.path.join(BASEDIR, "data", "sim", "objects.pkl"), "rb"
        ) as pkl_file:
            objects, robot_mdl = pickle.load(pkl_file)

    # Load config file
    cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

    # -----------------------------------

    # Set up planner interface and domain representation
    kb = KnowledgeBase(BASEDIR, domain_name="chimera")

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
        style=args.method, sleep_=args.sleep, load_objects=not restore_existing_objects,
    )
    scene = ScenePlanning1(world, BASEDIR, restored_objects=objects)

    # Spawn robot
    robot = RobotArm(world, cfg, BASEDIR, robot_mdl)
    robot.reset()

    robot.to_start()
    world.step_seconds(0.5)

    # Save world
    if not restore_existing_objects:
        savedir = os.path.join(BASEDIR, "data", "sim")
        if not os.path.isdir(savedir):
            os.makedirs(savedir)
        with open(os.path.join(savedir, "objects.pkl"), "wb") as output:
            pickle.dump((scene.objects, robot._model), output)
        p.saveBullet(os.path.join(savedir, "state.bullet"))

    # -----------------------------------

    # Set up predicates
    preds = Predicates(scene, robot, kb, cfg)
    kb.set_predicate_funcs(preds)

    for descr in preds.descriptions.items():
        kb.add_predicate(
            predicate_name=descr[0], predicate_definition=descr[1], overwrite=True
        )

    # Planning problem
    kb.populate_visible_objects(scene)
    kb.check_predicates()

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_place = SkillPlacing(scene, robot)
    sk_nav = SkillNavigate(scene, robot)
    skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

    # PDDL extender
    pddl_ex = PDDLExtender(kb, preds)

    # Set up exploration
    xplorer = Explorer(skill_set, robot, scene.objects, pddl_ex, kb, cfg)

    # Define a demonstration to guide exploration
    demo_sequence = ["place", "place"]
    demo_parameters = [{"obj": "lid1"}, {"obj": "cube1"}]

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
    sequence, parameters = plan

    if len(sequence) == 0:
        print("Nothing to do.")
        return
    print("---------------------------------------------------")
    print("Found plan:")
    for idx, seq_item in enumerate(sequence):
        print("".join((seq_item, " ", str(parameters[idx]))))
    print("---------------------------------------------------")
    input("Press enter to run...")

    # -----------------------------------

    # Set up execution system
    es = SequentialExecution(skill_set, sequence, parameters, kb)

    # Run
    try:
        index = 1
        while True:
            print("------------- Iteration {} ---------------".format(index))
            es.print_status()
            success, plan_finished, error_messages = es.step()
            if not success:
                print("Error messages:")
                for msg in error_messages:
                    print(msg)
                raise RuntimeError("Error during execution of current step. Aborting.")
            index += 1
            if plan_finished:
                print("Plan finished. Exiting.")
                break
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
