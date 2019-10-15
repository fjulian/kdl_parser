from sim.world import World
from sim.robot_arm import RobotArm

from sim.scene_tossing import SceneTossing
from sim.scene_planning_1 import ScenePlanning1

from skills.navigate import SkillNavigation
from skills.grasping import SkillGrasping

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    # Create world
    world = World(gui_=True, sleep_=True)
    scene = ScenePlanning1(world)

    # Spawn robot
    robot = RobotArm(world)
    robot.reset()

    # Set up skills
    sk_nav = SkillNavigation(scene, robot._model.uid)
    sk_grasp = SkillGrasping(scene, robot)

    # -----------------------------------
    
    robot.to_start()
    # world.step_seconds(30)
    world.step_seconds(1)

    # temp1 = p.getLinkState(robot._model.uid, robot.arm_base_link_idx)
    # r_O_O_rob = np.array(temp1[4])
    # goal_pos = np.array([0.4, 0.2, 0.2])
    # goal_orient = R.from_quat(robot.start_orient) #* R.from_euler("x",20,degrees=True)
    # world.draw_cross(goal_pos+r_O_O_rob)
    # robot.transition_cartesian(goal_pos, goal_orient.as_quat())
    # world.step_seconds(1)
    # goal_orient = R.from_euler("y",-90,degrees=True) * goal_orient
    # robot.transition_cartesian(goal_pos, goal_orient.as_quat())
    # world.step_seconds(1)
    # goal_orient = R.from_euler("x",20,degrees=True) * goal_orient
    # robot.transition_cartesian(goal_pos, goal_orient.as_quat())
    # world.step_seconds(1)

    # Robot velocity control
    # robot.update_velocity([0.1, 0.0, 0.0], 0.1)

    # Run move skill
    # res = sk_nav.move_to_object("cube1")
    res = sk_nav.move_to_object("cupboard")

    # Grasp the cube
    # sk_grasp.grasp_object("cube1")

    # Grasp the cupboard handle
    sk_grasp.grasp_object("cupboard", scene.objects["cupboard"].grasp_links[3])

    # Drive back
    robot.update_velocity([0.0, -0.1, 0.0], 0.0)
    world.step_seconds(4)
    robot.stop_driving()

    # Release
    sk_grasp.release_object()
    robot.to_start()

    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
