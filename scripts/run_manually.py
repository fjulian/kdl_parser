from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm

from highlevel_planning.sim.scene_tossing import SceneTossing
from highlevel_planning.sim.scene_planning_1 import ScenePlanning1
from highlevel_planning.sim.scene_move_skill import SceneMoveSkill

from highlevel_planning.skills.navigate import SkillNavigate
from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.placing import SkillPlacing
from highlevel_planning.skills.move import SkillMove

from highlevel_planning.knowledge.predicates import Predicates

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import pickle
import os

import argparse


def drawer_example(sk_grasp, sk_nav, robot, scene, world):
    # Run move skill
    sk_nav.move_to_object("cupboard")

    print(robot.get_wrist_force_torque())

    # Grasp the cupboard handle
    res = sk_grasp.grasp_object("cupboard", scene.objects["cupboard"].grasp_links[3])
    if not res:
        print("Grasping the handle failed.")
        return

    # Drive back
    robot.update_velocity([-0.1, 0.0, 0.0], 0.0)
    world.step_seconds(2)
    print(robot.get_wrist_force_torque())
    world.step_seconds(2)
    robot.stop_driving()

    # Release
    sk_grasp.release_object()
    robot.to_start()


def drawer_example_auto(sk_grasp, sk_nav, sk_move, robot, scene):
    # Run move skill
    sk_nav.move_to_object("cupboard", nav_min_dist=1.0)

    # Grasp the cupboard handle
    res = sk_grasp.grasp_object("cupboard", scene.objects["cupboard"].grasp_links[3])
    if not res:
        print("Grasping the handle failed.")
        return

    # Run the move skill
    sk_move.move_object(0.2, np.array([-1.0, 0.5, 0.0]))
    # sk_move.move_object(0.3, np.array([-0.5, 0.0, -0.5]))     # GT

    # Release
    sk_grasp.release_object()
    robot.to_start()


def cube_example(sk_grasp, sk_nav, robot, scene, sk_place):
    # Run move skill
    sk_nav.move_to_object("cube1")

    print("empty hand:")
    robot._world.step_seconds(1.0)
    print(robot.get_wrist_force_torque())

    # Grasp the cube
    sk_grasp.grasp_object("cube1")

    robot.to_start()
    print("after homing:")
    robot._world.step_seconds(1.0)
    print(robot.get_wrist_force_torque())

    # from math import pi as m_pi

    # robot.transition_cmd_to(
    #     np.array([0, -m_pi / 4.0, 0, -3.0 * m_pi / 4.0, 0, m_pi / 4.0, m_pi / 4.0])
    # )
    # print("after tilting back:")
    # robot._world.step_seconds(1.0)
    # print(robot.get_wrist_force())

    # robot.transition_cmd_to(
    #     np.array([0, -m_pi / 4.0, 0, -3.0 * m_pi / 4.0, 0, m_pi, m_pi / 4.0])
    # )
    # print("after tilting forward:")
    # robot._world.step_seconds(1.0)
    # print(robot.get_wrist_force())

    # robot.transition_cmd_to(
    #     np.array([0, -m_pi / 4.0, 0, -3.0 * m_pi / 4.0, m_pi / 2.0, m_pi, m_pi / 4.0])
    # )
    # print("after rolling:")
    # robot._world.step_seconds(1.0)
    # print(robot.get_wrist_force())

    # Place cube somewhere else
    sk_place.place_object(scene.objects["cube1"].init_pos + np.array([0.0, 0.2, -0.09]))


def navigate_with_cube(sk_nav, sk_grasp):
    # Run move skill
    sk_nav.move_to_object("cube1")

    # Grasp the cube
    sk_grasp.grasp_object("cube1")

    sk_nav.move_to_pos(np.array([0.0, 0.0, 0.0]), nav_min_dist=0.3)


def navigation_example(sk_nav, world):
    sk_nav.move_to_object("cube1")
    world.step_seconds(1.5)
    sk_nav.move_to_object("lid1")
    world.step_seconds(1.5)
    sk_nav.move_to_object("cupboard")
    world.step_seconds(1.5)
    sk_nav.move_to_object("container2")
    world.step_seconds(1.5)
    sk_nav.move_to_object("cube1")
    world.step_seconds(1.5)
    sk_nav.move_to_object("table")


def drive_example(robot, world):
    robot.update_velocity([0.4, 0.0, 0.0], 0.17)
    world.step_seconds(10)
    robot.stop_driving()


def predicate_example(scene, robot):
    preds = Predicates(scene, robot)
    print("Cube on table: " + str(preds.on("table", "cube1")))
    print("Cube on box 1: " + str(preds.on("container1", "cube1")))


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

    # Create world
    world = World(gui_=True, sleep_=False, load_objects=not restore_existing_objects)
    scene = ScenePlanning1(world, restored_objects=objects)
    # scene = SceneMoveSkill(world, restored_objects=objects)

    # Spawn robot
    robot = RobotArm(world, robot_mdl)
    robot.reset()

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot)
    sk_place = SkillPlacing(scene, robot)
    sk_nav = SkillNavigate(scene, robot)
    sk_move = SkillMove(scene, robot, 0.02, world.T_s)

    robot.to_start()
    world.step_seconds(0.5)

    # Save world
    if not restore_existing_objects:
        savedir = os.path.join(os.getcwd(), "data", "sim")
        if not os.path.isdir(savedir):
            os.makedirs(savedir)
        with open(os.path.join(savedir, "objects.pkl"), "wb") as output:
            pickle.dump((scene.objects, robot._model), output)
        p.saveBullet(os.path.join(savedir, "state.bullet"))

    # ---------- Run examples -----------

    # drawer_example(sk_grasp, sk_nav, robot, scene, world)

    drawer_example_auto(sk_grasp, sk_nav, sk_move, robot, scene)

    # cube_example(sk_grasp, sk_nav, robot, scene, sk_place)

    # drive_example(robot, world)

    # predicate_example(scene, robot)

    # navigate_with_cube(sk_nav, sk_grasp)

    # navigation_example(sk_nav, world)

    # -----------------------------------

    world.step_seconds(2)


if __name__ == "__main__":
    main()
