from sim.world import World
from sim.robot_arm import RobotArm


from sim.scene_planning_1 import ScenePlanning1

from skills.navigate import SkillNavigation
# from skills.grasping import SkillGrasping
from execution.bt import ExecutionSystem

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R

import py_trees

def main():
    # Create world
    world = World(gui_=True, sleep_=True)
    scene = ScenePlanning1(world)

    # Spawn robot
    robot = RobotArm(world)
    robot.reset()

    # Set up skills
    sk_nav = SkillNavigation(scene, robot._model.uid)
    # sk_grasp = SkillGrasping(scene, robot)

    # Set up behavior tree
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    es = ExecutionSystem(scene, robot)

    blackboard = py_trees.blackboard.Blackboard()
    blackboard.grasp_target_name = "cube1"
    blackboard.grasp_target_link_id = None
    blackboard.grasp_target_grasp_id = None

    # -----------------------------------
    
    robot.to_start()
    # world.step_seconds(30)
    world.step_seconds(1)

    # Run move skill
    res = sk_nav.move_to_object("cube1")
    # res = sk_nav.move_to_object("cupboard")

    # Grasp the cube
    # sk_grasp.grasp_object("cube1")


    # Grasp the cupboard handle
    # sk_grasp.grasp_object("cupboard", scene.objects["cupboard"].grasp_links[3])

    # Drive back
    # robot.update_velocity([0.0, -0.1, 0.0], 0.0)
    # world.step_seconds(4)
    # robot.stop_driving()

    # Release
    # sk_grasp.release_object()
    # robot.to_start()

    # res = sk_nav.move_to_object("cube1")
    # sk_grasp.grasp_object("cube1")

    try:
        es.tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)
    except KeyboardInterrupt:
        es.tree.interrupt() 

    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
