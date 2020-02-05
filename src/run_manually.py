from sim.world import World
from sim.robot_arm import RobotArm

from sim.scene_tossing import SceneTossing
from sim.scene_planning_1 import ScenePlanning1
from sim.scene_move_skill import SceneMoveSkill

from skills.navigate import move_to_object
from skills.grasping import SkillGrasping
from skills.placing import SkillPlacing

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import pickle

import argparse


def drawer_example(sk_grasp, robot, scene, world):
    # Run move skill
    move_to_object("cupboard", scene, robot._model.uid)

    # robot.get_wrist_force()

    # Grasp the cupboard handle
    res = sk_grasp.grasp_object("cupboard", scene.objects["cupboard"].grasp_links[3])
    if not res:
        print("Grasping the handle failed.")
        return

    # Drive back
    robot.update_velocity([-0.1, 0.0, 0.0], 0.0)
    world.step_seconds(2)
    # robot.get_wrist_force()
    world.step_seconds(2)
    robot.stop_driving()

    # Release
    sk_grasp.release_object()
    robot.to_start()


def cube_example(sk_grasp, robot, scene, sk_place):
    # Run move skill
    move_to_object("cube1", scene, robot._model.uid)

    print("empty hand:")
    robot._world.step_seconds(1.0)
    print(robot.get_wrist_force())

    # Grasp the cube
    sk_grasp.grasp_object("cube1")

    robot.to_start()
    print("after homing:")
    robot._world.step_seconds(1.0)
    print(robot.get_wrist_force())

    from math import pi as m_pi

    robot.transition_cmd_to(
        np.array([0, -m_pi / 4.0, 0, -3.0 * m_pi / 4.0, 0, m_pi / 4.0, m_pi / 4.0])
    )
    print("after tilting back:")
    robot._world.step_seconds(1.0)
    print(robot.get_wrist_force())

    robot.transition_cmd_to(
        np.array([0, -m_pi / 4.0, 0, -3.0 * m_pi / 4.0, 0, m_pi, m_pi / 4.0])
    )
    print("after tilting forward:")
    robot._world.step_seconds(1.0)
    print(robot.get_wrist_force())

    robot.transition_cmd_to(
        np.array([0, -m_pi / 4.0, 0, -3.0 * m_pi / 4.0, m_pi / 2.0, m_pi, m_pi / 4.0])
    )
    print("after rolling:")
    robot._world.step_seconds(1.0)
    print(robot.get_wrist_force())

    # Place cube somewhere else
    sk_place.place_object(scene.objects["cube1"].init_pos + np.array([0.0, 0.2, 0.0]))


def drive_example(robot, world):
    robot.update_velocity([0.4, 0.0, 0.0], 0.17)
    world.step_seconds(10)
    robot.stop_driving()


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
    world = World(gui_=True, sleep_=True, load_objects=not restore_existing_objects)
    scene = ScenePlanning1(world, restored_objects=objects)
    # scene = SceneMoveSkill(world, restored_objects=objects)

    # Spawn robot
    robot = RobotArm(world, robot_mdl)
    robot.reset()

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot)
    sk_place = SkillPlacing(scene, robot)

    robot.to_start()
    world.step_seconds(0.5)

    # ---------- Run examples -----------

    # drawer_example(sk_grasp, robot, scene, world)

    cube_example(sk_grasp, robot, scene, sk_place)

    # drive_example(robot, world)

    # -----------------------------------

    world.step_seconds(50)


if __name__ == "__main__":
    main()
