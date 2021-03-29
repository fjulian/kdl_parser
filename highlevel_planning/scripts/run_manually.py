from highlevel_planning_py.sim.scene_planning_1 import ScenePlanning1
from highlevel_planning_py.sim.scene_move_skill import SceneMoveSkill
from highlevel_planning_py.skills.navigate import SkillNavigate
from highlevel_planning_py.skills.grasping import SkillGrasping
from highlevel_planning_py.skills.placing import SkillPlacing
from highlevel_planning_py.skills.move import SkillMove
from highlevel_planning_py.knowledge.predicates import Predicates
from highlevel_planning_py.tools.config import ConfigYaml
from highlevel_planning_py.tools import run_util

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import os


SRCROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATHS = {
    "data_dir": os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning"),
    "src_root_dir": SRCROOT,
    "asset_dir": os.path.join(SRCROOT, "data", "models"),
    "bin_dir": os.path.join(SRCROOT, "bin"),
}


def drawer_example(sk_grasp, sk_nav, robot, world):
    # Run move skill
    sk_nav.move_to_object("cupboard")

    print(robot.get_wrist_force_torque())

    # Grasp the cupboard handle
    res = sk_grasp.grasp_object("cupboard", link_idx=3)
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
    res = sk_grasp.grasp_object("cupboard", link_idx=3)
    if not res:
        print("Grasping the handle failed.")
        return

    # Run the move skill
    sk_move.move_object(0.3, np.array([-1.0, 0.5, 0.0]))
    # sk_move.move_object(0.3, np.array([-0.5, 0.0, -0.5]))     # GT

    # Release
    sk_grasp.release_object()
    robot.to_start()


def grasp_example(
    sk_grasp,
    sk_nav,
    robot,
    scene,
    sk_place,
    object_name="cube1",
    link_idx=0,
    grasp_id=0,
):
    # Run nav skill
    sk_nav.move_to_object(object_name)

    # print("empty hand:")
    # robot._world.step_seconds(1.0)
    # print(robot.get_wrist_force_torque())

    # Grasp the object
    sk_grasp.grasp_object(object_name, link_idx=link_idx, grasp_id=grasp_id)

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
    new_pos = scene.objects[object_name].init_pos + np.array([0.2, 0.4, 0.05])
    sk_nav.move_to_pos(new_pos)
    sk_place.place_object(new_pos)

    robot.to_start()
    robot._world.step_seconds(1.0)


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
    args = run_util.parse_arguments()

    # Load existing simulation data if desired
    savedir = os.path.join(PATHS["data_dir"], "data", "sim")
    objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

    # Load config file
    cfg = ConfigYaml(os.path.join(SRCROOT, "config", "main.yaml"))

    # Create world
    scene, world = run_util.setup_pybullet_world(
        ScenePlanning1, PATHS["asset_dir"], args, savedir, objects
    )
    robot = run_util.setup_robot(world, cfg, PATHS["asset_dir"], robot_mdl)

    # Save state
    run_util.save_pybullet_sim(args, savedir, scene, robot)

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_place = SkillPlacing(scene, robot)
    sk_nav = SkillNavigate(scene, robot)
    sk_move = SkillMove(scene, robot, 0.02, robot._world.T_s)

    # bbox = np.array(p.getAABB(robot.model.uid))
    # for link in robot.model.link_name_to_index:
    #     this_box = np.array(
    #         p.getAABB(robot.model.uid, robot.model.link_name_to_index[link])
    #     )
    #     bbox[0, :] = np.minimum(bbox[0, :], this_box[0, :])
    #     bbox[1, :] = np.maximum(bbox[1, :], this_box[1, :])

    # ---------- Run examples -----------

    robot._world.step_seconds(1)

    # drawer_example(sk_grasp, sk_nav, robot, robot._world)

    # drawer_example_auto(sk_grasp, sk_nav, sk_move, robot, scene)

    # grasp_example(
    #     sk_grasp, sk_nav, robot, scene, sk_place, object_name="cube1", grasp_id=1
    # )
    # grasp_example(sk_grasp, sk_nav, robot, scene, sk_place, object_name="lid1")
    # grasp_example(sk_grasp, sk_nav, robot, scene, sk_place, object_name="lego")
    # grasp_example(sk_grasp, sk_nav, robot, scene, sk_place, object_name="duck")
    grasp_example(
        sk_grasp, sk_nav, robot, scene, sk_place, object_name="tall_box", grasp_id=1
    )

    # drive_example(robot, world)

    # predicate_example(scene, robot)

    # navigate_with_cube(sk_nav, sk_grasp)

    # navigation_example(sk_nav, world)

    # -----------------------------------

    robot._world.step_seconds(2)


if __name__ == "__main__":
    main()
